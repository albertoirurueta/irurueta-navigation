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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.NonSymmetricPositiveDefiniteMatrixException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.Accuracy3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.*;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

public class NonLinearFingerprintPositionAndRadioSourceEstimator3DTest implements
        FingerprintPositionAndRadioSourceEstimatorListener<Point3D> {

    private static final Logger LOGGER = Logger.getLogger(
            NonLinearFingerprintPositionAndRadioSourceEstimator3DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final int MIN_SOURCES = 10;
    private static final int MAX_SOURCES = 20;

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

    private static final double INITIAL_POSITION_ERROR_STD = 1.0;

    private static final double RSSI_ERROR_STD = 1.0e-1;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int TIMES = 50;

    private int estimateStart;
    private int estimateEnd;

    public NonLinearFingerprintPositionAndRadioSourceEstimator3DTest() { }

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
        NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), -1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getNumberOfDimensions(), 3);
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


        //test constructor with listener
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                this);

        //check default values
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertEquals(estimator.getMinNearestFingerprints(), -1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertFalse(estimator.isReady());
        assertEquals(estimator.getNumberOfDimensions(), 3);
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

        RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>
                locatedFingerprint = new RssiFingerprintLocated3D<>(readings,
                Point3D.create());

        List<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>
                locatedFingerprints = new ArrayList<>();
        locatedFingerprints.add(locatedFingerprint);

        RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                new RssiFingerprint<>(readings);


        //test constructor with located fingerprints and fingerprint
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                locatedFingerprints, fingerprint);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), -1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getNumberOfDimensions(), 3);
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

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                    null, fingerprint);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                    locatedFingerprints, null);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint and listener
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                locatedFingerprints, fingerprint, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), -1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getNumberOfDimensions(), 3);
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

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                    null, fingerprint, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                    locatedFingerprints, null, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint and initial position
        Point3D initialPosition = Point3D.create();

        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                locatedFingerprints, fingerprint, initialPosition);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), -1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getNumberOfDimensions(), 3);
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

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                    null, fingerprint, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                    locatedFingerprints, null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint, initial position
        //and listener
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                locatedFingerprints, fingerprint, initialPosition, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), -1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getNumberOfDimensions(), 3);
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

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                    null, fingerprint, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                    locatedFingerprints, null, initialPosition);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint and
        //initial located sources
        List<RadioSourceLocated<Point3D>> initialLocatedSources = new ArrayList<>();

        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                locatedFingerprints, fingerprint, initialLocatedSources);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), -1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getNumberOfDimensions(), 3);
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

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                    null, fingerprint, initialLocatedSources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                    locatedFingerprints, null, initialLocatedSources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint and initial located sources
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                locatedFingerprints, fingerprint, initialLocatedSources, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), -1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getNumberOfDimensions(), 3);
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

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                    null, fingerprint, initialLocatedSources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                    locatedFingerprints, null, initialLocatedSources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint, initial position and
        //initial located sources
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                locatedFingerprints, fingerprint, initialPosition,
                initialLocatedSources);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), -1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertNull(estimator.getListener());
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getNumberOfDimensions(), 3);
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

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                    null, fingerprint, initialPosition,
                    initialLocatedSources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                    locatedFingerprints, null, initialPosition,
                    initialLocatedSources);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);


        //test constructor with located fingerprints, fingerprint, initial position,
        //initial located sources and listener
        estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                locatedFingerprints, fingerprint, initialPosition,
                initialLocatedSources, this);

        //check default values
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertEquals(estimator.getMinNearestFingerprints(), -1);
        assertEquals(estimator.getMaxNearestFingerprints(), -1);
        assertEquals(estimator.getPathLossExponent(), 2.0, 0.0);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getEstimatedPositionCoordinates());
        assertNull(estimator.getEstimatedPosition());
        assertNull(estimator.getEstimatedLocatedSources());
        assertNull(estimator.getNearestFingerprints());
        assertFalse(estimator.isLocked());
        assertTrue(estimator.isReady());
        assertEquals(estimator.getNumberOfDimensions(), 3);
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

        //force IllegalArgumentException
        estimator = null;
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                    null, fingerprint, initialPosition,
                    initialLocatedSources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        try {
            estimator = new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                    locatedFingerprints, null, initialPosition,
                    initialLocatedSources, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (IllegalArgumentException ignore) { }
        assertNull(estimator);
    }

    @Test
    public void testGetSetLocatedFingerprints() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        //check default value
        assertNull(estimator.getLocatedFingerprints());

        //set new value
        List<RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>>
                locatedFingerprints = new ArrayList<>();
        estimator.setLocatedFingerprints(locatedFingerprints);

        //check
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
    }

    @Test
    public void testGetSetFingerprint() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

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
        NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        //check default values
        assertEquals(estimator.getMinNearestFingerprints(), -1);
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
        NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

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
        NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        //check default value
        assertNull(estimator.getListener());

        //set new value
        estimator.setListener(this);

        //check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testGetSetUseNoMeanNearestFingerprintFinder() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        //check default value
        assertTrue(estimator.getUseNoMeanNearestFingerprintFinder());

        //set new value
        estimator.setUseNoMeanNearestFingerprintFinder(false);

        //check
        assertFalse(estimator.getUseNoMeanNearestFingerprintFinder());
    }

    @Test
    public void testGetSetInitialLocatedSources() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        //check default value
        assertNull(estimator.getInitialLocatedSources());

        //set new value
        List<RadioSourceLocated<Point3D>> initialLocatedSources = new ArrayList<>();
        estimator.setInitialLocatedSources(initialLocatedSources);

        //check
        assertSame(estimator.getInitialLocatedSources(), initialLocatedSources);
    }

    @Test
    public void testGetSetInitialPosition() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        //check default value
        assertNull(estimator.getInitialPosition());

        //set new value
        Point3D initialPosition = Point3D.create();
        estimator.setInitialPosition(initialPosition);

        //check
        assertSame(estimator.getInitialPosition(), initialPosition);
    }

    @Test
    public void testGetSetUseSourcesPathLossExponentWhenAvailable() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        //check default value
        assertTrue(estimator.getUseSourcesPathLossExponentWhenAvailable());

        //set new value
        estimator.setUseSourcesPathLossExponentWhenAvailable(false);

        //check
        assertFalse(estimator.getUseSourcesPathLossExponentWhenAvailable());
    }

    @Test
    public void testGetSetFallbackRssiStandardDeviation() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

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
        NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        //check default value
        assertTrue(estimator.isFingerprintRssiStandardDeviationPropagated());

        //set new value
        estimator.setFingerprintRssiStandardDeviationPropagated(false);

        //check
        assertFalse(estimator.isFingerprintRssiStandardDeviationPropagated());
    }

    @Test
    public void testIsSetPathlossExponentStandardDeviationPropagated() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        //check default value
        assertTrue(estimator.isPathlossExponentStandardDeviationPropagated());

        //set new value
        estimator.setPathlossExponentStandardDeviationPropagated(false);

        //check
        assertFalse(estimator.isPathlossExponentStandardDeviationPropagated());
    }

    @Test
    public void testIsSetFingerprintPositionCovariancePropagated() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        //check default value
        assertTrue(estimator.isFingerprintPositionCovariancePropagated());

        //set new value
        estimator.setFingerprintPositionCovariancePropagated(false);

        //check
        assertFalse(estimator.isFingerprintPositionCovariancePropagated());
    }

    @Test
    public void testIsSetRadioSourcePositionCovariancePropagated() throws LockedException {
        NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                new NonLinearFingerprintPositionAndRadioSourceEstimator3D();

        //check default value
        assertTrue(estimator.isRadioSourcePositionCovariancePropagated());

        //set new value
        estimator.setRadioSourcePositionCovariancePropagated(false);

        //check
        assertFalse(estimator.isRadioSourcePositionCovariancePropagated());
    }

    @Test
    public void testEstimateOneRadioSourceWithoutInitialValues()
            throws LockedException, FingerprintEstimationException, NotReadyException {

        Accuracy3D accuracy = new Accuracy3D();

        double avgPositionError = 0.0;
        double avgSourcePositionError = 0.0;
        double avgPositionAccuracy = 0.0;
        double avgSourceAccuracy = 0.0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            //build sources
            int numSources = 1;
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for(int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                                transmittedPowerdBm, position);
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
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
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

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //create estimator
            NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                    new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                            locatedFingerprints, fingerprint,this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check results
            Point3D estimatedPosition = estimator.getEstimatedPosition();
            Matrix estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            List<RadioSourceLocated<Point3D>> estimatedLocatedSources =
                    estimator.getEstimatedLocatedSources();
            Matrix estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            double positionError = estimatedPosition.distanceTo(position);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                double positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            double avgSourcePositionErrorOneTime = 0.0;
            double avgSourceAccuracyOneTime = 0.0;
            int numValidSources = 0;
            for (RadioSourceLocated<Point3D> estimatedLocatedSource : estimatedLocatedSources) {
                Point3D estimatedSourcePosition = estimatedLocatedSource.getPosition();
                Matrix estimatedSourcePositionCovariance =
                        estimatedLocatedSource.getPositionCovariance();

                int index = sources.indexOf(estimatedLocatedSource);
                RadioSourceLocated<Point3D> source = sources.get(index);
                Point3D sourcePosition = source.getPosition();
                double sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    double sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        double percentage = (double)numValid / (double)TIMES;


        LOGGER.log(Level.INFO, "Valid percentage: {0}%",
                percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m",
                avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m",
                avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m",
                avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m",
                avgSourceAccuracy);

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testEstimateOneRadioSourceWithExactInitialValues()
            throws LockedException, FingerprintEstimationException, NotReadyException {

        Accuracy3D accuracy = new Accuracy3D();

        double avgPositionError = 0.0;
        double avgSourcePositionError = 0.0;
        double avgPositionAccuracy = 0.0;
        double avgSourceAccuracy = 0.0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            //build sources
            int numSources = 1;
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for(int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                                transmittedPowerdBm, position);
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
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
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

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //create estimator
            NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                    new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                            locatedFingerprints, fingerprint,
                            position,
                            sources,
                            this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            Matrix estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            List<RadioSourceLocated<Point3D>> estimatedLocatedSources =
                    estimator.getEstimatedLocatedSources();
            Matrix estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            double positionError = estimatedPosition.distanceTo(position);

            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(positionError <= ABSOLUTE_ERROR);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                double positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            double avgSourcePositionErrorOneTime = 0.0;
            double avgSourceAccuracyOneTime = 0.0;
            int numValidSources = 0;
            for (RadioSourceLocated<Point3D> estimatedLocatedSource : estimatedLocatedSources) {
                Point3D estimatedSourcePosition = estimatedLocatedSource.getPosition();
                Matrix estimatedSourcePositionCovariance =
                        estimatedLocatedSource.getPositionCovariance();

                int index = sources.indexOf(estimatedLocatedSource);
                RadioSourceLocated<Point3D> source = sources.get(index);
                Point3D sourcePosition = source.getPosition();
                double sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                if (sourcePositionError > ABSOLUTE_ERROR) {
                    continue;
                }
                assertTrue(sourcePositionError <= ABSOLUTE_ERROR);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    double sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        double percentage = (double)numValid / (double)TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%",
                percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m",
                avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m",
                avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m",
                avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m",
                avgSourceAccuracy);

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testEstimateOneRadioSourceWithApproximateInitialValues()
            throws LockedException, FingerprintEstimationException, NotReadyException {

        Accuracy3D accuracy = new Accuracy3D();

        double avgPositionError = 0.0;
        double avgSourcePositionError = 0.0;
        double avgPositionAccuracy = 0.0;
        double avgSourceAccuracy = 0.0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            GaussianRandomizer positionErrorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, INITIAL_POSITION_ERROR_STD);

            //build sources
            int numSources = 1;
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            List<RadioSourceLocated<Point3D>> initialSources = new ArrayList<>();
            for(int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                        x + positionErrorRandomizer.nextDouble(),
                        y + positionErrorRandomizer.nextDouble(),
                        z + positionErrorRandomizer.nextDouble());

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                                transmittedPowerdBm, position);
                sources.add(accessPoint);

                WifiAccessPointWithPowerAndLocated3D initialAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                                transmittedPowerdBm, initialPosition);
                initialSources.add(initialAccessPoint);
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
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
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

            InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                    x + positionErrorRandomizer.nextDouble(),
                    y + positionErrorRandomizer.nextDouble(),
                    z + positionErrorRandomizer.nextDouble());

            List<RssiReading<RadioSource>> readings = new ArrayList<>();
            for (RadioSourceLocated<Point3D> source : sources) {
                double distance = source.getPosition().distanceTo(position);
                double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).
                        getTransmittedPower();

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //create estimator
            NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                    new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                            locatedFingerprints, fingerprint, initialPosition,
                            initialSources,this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check results
            Point3D estimatedPosition = estimator.getEstimatedPosition();
            Matrix estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            List<RadioSourceLocated<Point3D>> estimatedLocatedSources =
                    estimator.getEstimatedLocatedSources();
            Matrix estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            double positionError = estimatedPosition.distanceTo(position);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                double positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            double avgSourcePositionErrorOneTime = 0.0;
            double avgSourceAccuracyOneTime = 0.0;
            int numValidSources = 0;
            for (RadioSourceLocated<Point3D> estimatedLocatedSource : estimatedLocatedSources) {
                Point3D estimatedSourcePosition = estimatedLocatedSource.getPosition();
                Matrix estimatedSourcePositionCovariance =
                        estimatedLocatedSource.getPositionCovariance();

                int index = sources.indexOf(estimatedLocatedSource);
                RadioSourceLocated<Point3D> source = sources.get(index);
                Point3D sourcePosition = source.getPosition();
                double sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    double sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        double percentage = (double)numValid / (double)TIMES;


        LOGGER.log(Level.INFO, "Valid percentage: {0}%",
                percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m",
                avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m",
                avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m",
                avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m",
                avgSourceAccuracy);

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testEstimateOneRadioSourceWithNonLocatedSources()
            throws LockedException, FingerprintEstimationException, NotReadyException {

        Accuracy3D accuracy = new Accuracy3D();

        double avgPositionError = 0.0;
        double avgSourcePositionError = 0.0;
        double avgPositionAccuracy = 0.0;
        double avgSourceAccuracy = 0.0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            //build sources
            int numSources = 1;
            List<RadioSourceLocated<Point3D>> locatedSources = new ArrayList<>();
            List<RadioSource> sources = new ArrayList<>();
            for(int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                WifiAccessPointWithPowerAndLocated3D locatedAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                                transmittedPowerdBm, position);
                locatedSources.add(locatedAccessPoint);

                WifiAccessPoint accessPoint = new WifiAccessPoint("bssid" + i, FREQUENCY);
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
                int k = 0;
                for (RadioSourceLocated<Point3D> locatedSource : locatedSources) {
                    double distance = locatedSource.getPosition().distanceTo(position);
                    double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) locatedSource).
                            getTransmittedPower();

                    double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));

                    RadioSource source = sources.get(k);
                    RssiReading<RadioSource> reading = new RssiReading<>(source,
                            receivedRssi);
                    readings.add(reading);
                    k++;
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
            int k = 0;
            for (RadioSourceLocated<Point3D> locatedSource : locatedSources) {
                double distance = locatedSource.getPosition().distanceTo(position);
                double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) locatedSource).
                        getTransmittedPower();

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));

                RadioSource source = sources.get(k);
                RssiReading<RadioSource> reading = new RssiReading<>(source,
                        receivedRssi);
                readings.add(reading);
                k++;
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //create estimator
            NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                    new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                            locatedFingerprints, fingerprint, position, locatedSources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            Matrix estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            List<RadioSourceLocated<Point3D>> estimatedLocatedSources =
                    estimator.getEstimatedLocatedSources();
            Matrix estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            double positionError = estimatedPosition.distanceTo(position);

            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(positionError <= ABSOLUTE_ERROR);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                double positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            double avgSourcePositionErrorOneTime = 0.0;
            double avgSourceAccuracyOneTime = 0.0;
            int numValidSources = 0;
            for (RadioSourceLocated<Point3D> estimatedLocatedSource : estimatedLocatedSources) {
                Point3D estimatedSourcePosition = estimatedLocatedSource.getPosition();
                Matrix estimatedSourcePositionCovariance =
                        estimatedLocatedSource.getPositionCovariance();

                int index = locatedSources.indexOf(estimatedLocatedSource);
                RadioSourceLocated<Point3D> source = locatedSources.get(index);
                Point3D sourcePosition = source.getPosition();
                double sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                if (sourcePositionError > ABSOLUTE_ERROR) {
                    continue;
                }
                assertTrue(sourcePositionError <= ABSOLUTE_ERROR);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    double sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        double percentage = (double)numValid / (double)TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%",
                percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m",
                avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m",
                avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m",
                avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m",
                avgSourceAccuracy);

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testEstimateOneRadioSourceWithRssiError()
            throws LockedException, FingerprintEstimationException, NotReadyException {

        Accuracy3D accuracy = new Accuracy3D();

        double avgPositionError = 0.0;
        double avgSourcePositionError = 0.0;
        double avgPositionAccuracy = 0.0;
        double avgSourceAccuracy = 0.0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            GaussianRandomizer rssiErrorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, RSSI_ERROR_STD);

            //build sources
            int numSources = 1;
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for(int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                                transmittedPowerdBm, position);
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
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                    double rssiError = rssiErrorRandomizer.nextDouble();

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

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                double rssiError = rssiErrorRandomizer.nextDouble();

                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi + rssiError);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //create estimator
            NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                    new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                            locatedFingerprints, fingerprint, position,
                            sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            Matrix estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            List<RadioSourceLocated<Point3D>> estimatedLocatedSources =
                    estimator.getEstimatedLocatedSources();
            Matrix estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            double positionError = estimatedPosition.distanceTo(position);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                double positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            double avgSourcePositionErrorOneTime = 0.0;
            double avgSourceAccuracyOneTime = 0.0;
            int numValidSources = 0;
            for (RadioSourceLocated<Point3D> estimatedLocatedSource : estimatedLocatedSources) {
                Point3D estimatedSourcePosition = estimatedLocatedSource.getPosition();
                Matrix estimatedSourcePositionCovariance =
                        estimatedLocatedSource.getPositionCovariance();

                int index = sources.indexOf(estimatedLocatedSource);
                RadioSourceLocated<Point3D> source = sources.get(index);
                Point3D sourcePosition = source.getPosition();
                double sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    double sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        double percentage = (double)numValid / (double)TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%",
                percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m",
                avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m",
                avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m",
                avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m",
                avgSourceAccuracy);

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testEstimateOneRadioSourceWithBias()
            throws LockedException, FingerprintEstimationException, NotReadyException {

        Accuracy3D accuracy = new Accuracy3D();

        double avgPositionError = 0.0;
        double avgSourcePositionError = 0.0;
        double avgPositionAccuracy = 0.0;
        double avgSourceAccuracy = 0.0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            //build sources
            int numSources = 1;
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for(int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                                transmittedPowerdBm, position);
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
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                    RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                            receivedRssi + RSSI_BIAS);
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

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi + RSSI_BIAS);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //create estimator
            NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                    new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                            locatedFingerprints, fingerprint, position,
                            sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            Matrix estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            List<RadioSourceLocated<Point3D>> estimatedLocatedSources =
                    estimator.getEstimatedLocatedSources();
            Matrix estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            double positionError = estimatedPosition.distanceTo(position);

            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(positionError <= ABSOLUTE_ERROR);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                double positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            double avgSourcePositionErrorOneTime = 0.0;
            double avgSourceAccuracyOneTime = 0.0;
            int numValidSources = 0;
            for (RadioSourceLocated<Point3D> estimatedLocatedSource : estimatedLocatedSources) {
                Point3D estimatedSourcePosition = estimatedLocatedSource.getPosition();
                Matrix estimatedSourcePositionCovariance =
                        estimatedLocatedSource.getPositionCovariance();

                int index = sources.indexOf(estimatedLocatedSource);
                RadioSourceLocated<Point3D> source = sources.get(index);
                Point3D sourcePosition = source.getPosition();
                double sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                if (sourcePositionError > ABSOLUTE_ERROR) {
                    continue;
                }
                assertTrue(sourcePositionError <= ABSOLUTE_ERROR);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    double sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        double percentage = (double)numValid / (double)TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%",
                percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m",
                avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m",
                avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m",
                avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m",
                avgSourceAccuracy);

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testEstimateOneRadioSourceWithRssiErrorAndBias()
            throws LockedException, FingerprintEstimationException, NotReadyException {

        Accuracy3D accuracy = new Accuracy3D();

        double avgPositionError = 0.0;
        double avgSourcePositionError = 0.0;
        double avgPositionAccuracy = 0.0;
        double avgSourceAccuracy = 0.0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            GaussianRandomizer rssiErrorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, RSSI_ERROR_STD);

            //build sources
            int numSources = 1;
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for(int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                                transmittedPowerdBm, position);
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
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                    double rssiError = rssiErrorRandomizer.nextDouble();

                    RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                            receivedRssi + rssiError + RSSI_BIAS);
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

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                double rssiError = rssiErrorRandomizer.nextDouble();

                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi + rssiError + RSSI_BIAS);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //create estimator
            NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                    new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                            locatedFingerprints, fingerprint,
                            position,
                            sources,
                            this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            Matrix estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            List<RadioSourceLocated<Point3D>> estimatedLocatedSources =
                    estimator.getEstimatedLocatedSources();
            Matrix estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            double positionError = estimatedPosition.distanceTo(position);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                double positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            double avgSourcePositionErrorOneTime = 0.0;
            double avgSourceAccuracyOneTime = 0.0;
            int numValidSources = 0;
            for (RadioSourceLocated<Point3D> estimatedLocatedSource : estimatedLocatedSources) {
                Point3D estimatedSourcePosition = estimatedLocatedSource.getPosition();
                Matrix estimatedSourcePositionCovariance =
                        estimatedLocatedSource.getPositionCovariance();

                int index = sources.indexOf(estimatedLocatedSource);
                RadioSourceLocated<Point3D> source = sources.get(index);
                Point3D sourcePosition = source.getPosition();
                double sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    double sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        double percentage = (double)numValid / (double)TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%",
                percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m",
                avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m",
                avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m",
                avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m",
                avgSourceAccuracy);

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testEstimateOneRadioSourceWithVariancePropagation()
            throws LockedException, FingerprintEstimationException, NotReadyException {

        Accuracy3D accuracy = new Accuracy3D();

        double avgPositionError = 0.0;
        double avgSourcePositionError = 0.0;
        double avgPositionAccuracy = 0.0;
        double avgSourceAccuracy = 0.0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            //build sources
            int numSources = 1;
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for(int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                double positionStd = randomizer.nextDouble(
                        MIN_POSITION_STANDARD_DEVIATION,
                        MAX_POSITION_STANDARD_DEVIATION);
                double positionVariance = positionStd * positionStd;
                Matrix positionCovariance = Matrix.diagonal(
                        new double[] { positionVariance, positionVariance, positionVariance });
                double pathLossStd = randomizer.nextDouble(MIN_PATHLOSS_STANDARD_DEVIATION,
                        MAX_PATHLOSS_STANDARD_DEVIATION);

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(
                                "bssid" + i, FREQUENCY, transmittedPowerdBm,
                                null,
                                NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                                pathLossStd, position, positionCovariance);
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

                double positionStd = randomizer.nextDouble(
                        MIN_POSITION_STANDARD_DEVIATION,
                        MAX_POSITION_STANDARD_DEVIATION);
                double positionVariance = positionStd * positionStd;
                Matrix positionCovariance = Matrix.diagonal(
                        new double[] { positionVariance, positionVariance, positionVariance });

                List<RssiReading<RadioSource>> readings = new ArrayList<>();
                for (RadioSourceLocated<Point3D> source : sources) {
                    double distance = source.getPosition().distanceTo(position);
                    double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).
                            getTransmittedPower();

                    double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                    double receivedRssiStd = randomizer.nextDouble(
                            MIN_RSSI_STANDARD_DEVIATION, MAX_RSSI_STANDARD_DEVIATION);

                    RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                            receivedRssi, receivedRssiStd);
                    readings.add(reading);
                }

                RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> locatedFingerprint =
                        new RssiFingerprintLocated3D<>(readings, position,
                                positionCovariance);
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

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //create estimator
            NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                    new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                            locatedFingerprints, fingerprint, position,
                            sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            Matrix estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            List<RadioSourceLocated<Point3D>> estimatedLocatedSources =
                    estimator.getEstimatedLocatedSources();
            Matrix estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            double positionError = estimatedPosition.distanceTo(position);

            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(positionError <= ABSOLUTE_ERROR);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                double positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            double avgSourcePositionErrorOneTime = 0.0;
            double avgSourceAccuracyOneTime = 0.0;
            int numValidSources = 0;
            for (RadioSourceLocated<Point3D> estimatedLocatedSource : estimatedLocatedSources) {
                Point3D estimatedSourcePosition = estimatedLocatedSource.getPosition();
                Matrix estimatedSourcePositionCovariance =
                        estimatedLocatedSource.getPositionCovariance();

                int index = sources.indexOf(estimatedLocatedSource);
                RadioSourceLocated<Point3D> source = sources.get(index);
                Point3D sourcePosition = source.getPosition();
                double sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                if (sourcePositionError > ABSOLUTE_ERROR) {
                    continue;
                }
                assertTrue(sourcePositionError <= ABSOLUTE_ERROR);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    double sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        double percentage = (double)numValid / (double)TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%",
                percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m",
                avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m",
                avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m",
                avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m",
                avgSourceAccuracy);

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testEstimateBeacon()
            throws LockedException, FingerprintEstimationException, NotReadyException {

        Accuracy3D accuracy = new Accuracy3D();

        double avgPositionError = 0.0;
        double avgSourcePositionError = 0.0;
        double avgPositionAccuracy = 0.0;
        double avgSourceAccuracy = 0.0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            //build sources
            int numSources = 1;
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for(int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                BeaconIdentifier identifier = BeaconIdentifier.fromUuid(UUID.randomUUID());
                BeaconLocated3D beacon = new BeaconLocated3D(
                        Collections.singletonList(identifier),
                        transmittedPowerdBm, FREQUENCY, position);
                sources.add(beacon);
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
                    double transmittedPowerdBm = ((BeaconLocated3D) source).
                            getTransmittedPower();

                    double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
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
                double transmittedPowerdBm = ((BeaconLocated3D) source).
                        getTransmittedPower();

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //create estimator
            NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                    new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                            locatedFingerprints, fingerprint, position,
                            sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            Matrix estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            List<RadioSourceLocated<Point3D>> estimatedLocatedSources =
                    estimator.getEstimatedLocatedSources();
            Matrix estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            double positionError = estimatedPosition.distanceTo(position);

            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(positionError <= ABSOLUTE_ERROR);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                double positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            double avgSourcePositionErrorOneTime = 0.0;
            double avgSourceAccuracyOneTime = 0.0;
            int numValidSources = 0;
            for (RadioSourceLocated<Point3D> estimatedLocatedSource : estimatedLocatedSources) {
                Point3D estimatedSourcePosition = estimatedLocatedSource.getPosition();
                Matrix estimatedSourcePositionCovariance =
                        estimatedLocatedSource.getPositionCovariance();

                int index = sources.indexOf(estimatedLocatedSource);
                RadioSourceLocated<Point3D> source = sources.get(index);
                Point3D sourcePosition = source.getPosition();
                double sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                if (sourcePositionError > ABSOLUTE_ERROR) {
                    continue;
                }
                assertTrue(sourcePositionError <= ABSOLUTE_ERROR);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    double sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        double percentage = (double)numValid / (double)TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%",
                percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m",
                avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m",
                avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m",
                avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m",
                avgSourceAccuracy);

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testEstimateOneRadioSourceWithOtherPathLossExponent()
            throws LockedException, FingerprintEstimationException, NotReadyException {

        Accuracy3D accuracy = new Accuracy3D();

        double avgPositionError = 0.0;
        double avgSourcePositionError = 0.0;
        double avgPositionAccuracy = 0.0;
        double avgSourceAccuracy = 0.0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            double pathLossExponent = randomizer.nextDouble(
                    MIN_PATH_LOSS_EXPONENT, MAX_PATH_LOSS_EXPONENT);

            //build sources
            int numSources = 1;
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for(int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D("bssid" + i,
                                FREQUENCY, transmittedPowerdBm, pathLossExponent,
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

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, pathLossExponent));
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //create estimator
            NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                    new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                            locatedFingerprints, fingerprint, position,
                            sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setPathLossExponent(pathLossExponent);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            Matrix estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            List<RadioSourceLocated<Point3D>> estimatedLocatedSources =
                    estimator.getEstimatedLocatedSources();
            Matrix estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            double positionError = estimatedPosition.distanceTo(position);

            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(positionError <= ABSOLUTE_ERROR);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                double positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            double avgSourcePositionErrorOneTime = 0.0;
            double avgSourceAccuracyOneTime = 0.0;
            int numValidSources = 0;
            for (RadioSourceLocated<Point3D> estimatedLocatedSource : estimatedLocatedSources) {
                Point3D estimatedSourcePosition = estimatedLocatedSource.getPosition();
                Matrix estimatedSourcePositionCovariance =
                        estimatedLocatedSource.getPositionCovariance();

                int index = sources.indexOf(estimatedLocatedSource);
                RadioSourceLocated<Point3D> source = sources.get(index);
                Point3D sourcePosition = source.getPosition();
                double sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                if (sourcePositionError > ABSOLUTE_ERROR) {
                    continue;
                }
                assertTrue(sourcePositionError <= ABSOLUTE_ERROR);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    double sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        double percentage = (double)numValid / (double)TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%",
                percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m",
                avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m",
                avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m",
                avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m",
                avgSourceAccuracy);

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testEstimateTwoRadioSourcesWithoutInitialValues()
            throws LockedException, FingerprintEstimationException, NotReadyException {

        Accuracy3D accuracy = new Accuracy3D();

        double avgPositionError = 0.0;
        double avgSourcePositionError = 0.0;
        double avgPositionAccuracy = 0.0;
        double avgSourceAccuracy = 0.0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            //build sources
            int numSources = 2;
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for(int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                                transmittedPowerdBm, position);
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
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
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

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //create estimator
            NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                    new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                            locatedFingerprints, fingerprint,this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            //check results
            Point3D estimatedPosition = estimator.getEstimatedPosition();
            Matrix estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            List<RadioSourceLocated<Point3D>> estimatedLocatedSources =
                    estimator.getEstimatedLocatedSources();
            Matrix estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertEquals(estimatedLocatedSources.size(), numSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            double positionError = estimatedPosition.distanceTo(position);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                double positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            double avgSourcePositionErrorOneTime = 0.0;
            double avgSourceAccuracyOneTime = 0.0;
            int numValidSources = 0;
            for (RadioSourceLocated<Point3D> estimatedLocatedSource : estimatedLocatedSources) {
                Point3D estimatedSourcePosition = estimatedLocatedSource.getPosition();
                Matrix estimatedSourcePositionCovariance =
                        estimatedLocatedSource.getPositionCovariance();

                int index = sources.indexOf(estimatedLocatedSource);
                RadioSourceLocated<Point3D> source = sources.get(index);
                Point3D sourcePosition = source.getPosition();
                double sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    double sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        double percentage = (double)numValid / (double)TIMES;


        LOGGER.log(Level.INFO, "Valid percentage: {0}%",
                percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m",
                avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m",
                avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m",
                avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m",
                avgSourceAccuracy);

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testEstimateTwoRadioSourcesWithExactInitialValues()
            throws LockedException, FingerprintEstimationException, NotReadyException {

        Accuracy3D accuracy = new Accuracy3D();

        double avgPositionError = 0.0;
        double avgSourcePositionError = 0.0;
        double avgPositionAccuracy = 0.0;
        double avgSourceAccuracy = 0.0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            //build sources
            int numSources = 2;
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for(int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                                transmittedPowerdBm, position);
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
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
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

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //create estimator
            NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                    new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                            locatedFingerprints, fingerprint, position,
                            sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            Matrix estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            List<RadioSourceLocated<Point3D>> estimatedLocatedSources =
                    estimator.getEstimatedLocatedSources();
            Matrix estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertEquals(estimatedLocatedSources.size(), numSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            double positionError = estimatedPosition.distanceTo(position);

            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(positionError <= ABSOLUTE_ERROR);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                double positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            double avgSourcePositionErrorOneTime = 0.0;
            double avgSourceAccuracyOneTime = 0.0;
            int numValidSources = 0;
            for (RadioSourceLocated<Point3D> estimatedLocatedSource : estimatedLocatedSources) {
                Point3D estimatedSourcePosition = estimatedLocatedSource.getPosition();
                Matrix estimatedSourcePositionCovariance =
                        estimatedLocatedSource.getPositionCovariance();

                int index = sources.indexOf(estimatedLocatedSource);
                RadioSourceLocated<Point3D> source = sources.get(index);
                Point3D sourcePosition = source.getPosition();
                double sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                if (sourcePositionError > ABSOLUTE_ERROR) {
                    continue;
                }
                assertTrue(sourcePositionError <= ABSOLUTE_ERROR);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    double sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        double percentage = (double)numValid / (double)TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%",
                percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m",
                avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m",
                avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m",
                avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m",
                avgSourceAccuracy);

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testEstimateTwoRadioSourcesWithVariancePropagation()
            throws LockedException, FingerprintEstimationException, NotReadyException {

        Accuracy3D accuracy = new Accuracy3D();

        double avgPositionError = 0.0;
        double avgSourcePositionError = 0.0;
        double avgPositionAccuracy = 0.0;
        double avgSourceAccuracy = 0.0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            //build sources
            int numSources = 2;
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for(int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                double positionStd = randomizer.nextDouble(
                        MIN_POSITION_STANDARD_DEVIATION,
                        MAX_POSITION_STANDARD_DEVIATION);
                double positionVariance = positionStd * positionStd;
                Matrix positionCovariance = Matrix.diagonal(
                        new double[] { positionVariance, positionVariance, positionVariance });
                double pathLossStd = randomizer.nextDouble(MIN_PATHLOSS_STANDARD_DEVIATION,
                        MAX_PATHLOSS_STANDARD_DEVIATION);

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D(
                                "bssid" + i, FREQUENCY, transmittedPowerdBm,
                                null,
                                NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT,
                                pathLossStd, position, positionCovariance);
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

                double positionStd = randomizer.nextDouble(
                        MIN_POSITION_STANDARD_DEVIATION,
                        MAX_POSITION_STANDARD_DEVIATION);
                double positionVariance = positionStd * positionStd;
                Matrix positionCovariance = Matrix.diagonal(
                        new double[] { positionVariance, positionVariance, positionVariance });

                List<RssiReading<RadioSource>> readings = new ArrayList<>();
                for (RadioSourceLocated<Point3D> source : sources) {
                    double distance = source.getPosition().distanceTo(position);
                    double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).
                            getTransmittedPower();

                    double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                    double receivedRssiStd = randomizer.nextDouble(
                            MIN_RSSI_STANDARD_DEVIATION, MAX_RSSI_STANDARD_DEVIATION);

                    RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                            receivedRssi, receivedRssiStd);
                    readings.add(reading);
                }

                RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>> locatedFingerprint =
                        new RssiFingerprintLocated3D<>(readings, position,
                                positionCovariance);
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

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //create estimator
            NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                    new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                            locatedFingerprints, fingerprint,
                            position,
                            sources,
                            this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            Matrix estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            List<RadioSourceLocated<Point3D>> estimatedLocatedSources =
                    estimator.getEstimatedLocatedSources();
            Matrix estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            double positionError = estimatedPosition.distanceTo(position);

            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(positionError <= ABSOLUTE_ERROR);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                double positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            double avgSourcePositionErrorOneTime = 0.0;
            double avgSourceAccuracyOneTime = 0.0;
            int numValidSources = 0;
            for (RadioSourceLocated<Point3D> estimatedLocatedSource : estimatedLocatedSources) {
                Point3D estimatedSourcePosition = estimatedLocatedSource.getPosition();
                Matrix estimatedSourcePositionCovariance =
                        estimatedLocatedSource.getPositionCovariance();

                int index = sources.indexOf(estimatedLocatedSource);
                RadioSourceLocated<Point3D> source = sources.get(index);
                Point3D sourcePosition = source.getPosition();
                double sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                if (sourcePositionError > ABSOLUTE_ERROR) {
                    continue;
                }
                assertTrue(sourcePositionError <= ABSOLUTE_ERROR);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    double sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        double percentage = (double)numValid / (double)TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%",
                percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m",
                avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m",
                avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m",
                avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m",
                avgSourceAccuracy);

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testEstimateWithoutEnoughFingerprintsForARadioSource()
            throws LockedException, FingerprintEstimationException, NotReadyException {

        Accuracy3D accuracy = new Accuracy3D();

        double avgPositionError = 0.0;
        double avgSourcePositionError = 0.0;
        double avgPositionAccuracy = 0.0;
        double avgSourceAccuracy = 0.0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            //build sources
            int numSources = 2;
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for(int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                                transmittedPowerdBm, position);
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
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
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

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //find closest located fingerprint to actual position and leave only
            //dims - 1 readings for last radio source
            int minNearestFingerprints = 9;
            RadioSourceNoMeanKNearestFinder<Point3D, RadioSource> noMeanfinder =
                    new RadioSourceNoMeanKNearestFinder<>(locatedFingerprints);
            List<RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point3D>> nearestFingerprints =
                    noMeanfinder.findKNearestTo(fingerprint, minNearestFingerprints);
            RadioSourceLocated<Point3D> lastSource = sources.get(sources.size() - 1);
            int numReadingsLastSource = 0;
            locatedFingerprints.clear();

            for(RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point3D> nearFingerprint : nearestFingerprints) {
                List<RssiReading<RadioSource>> nearReadings = nearFingerprint.getReadings();

                List<RssiReading<RadioSource>> readingsToRemove = new ArrayList<>();
                for(RssiReading<RadioSource> nearReading : nearReadings) {
                    if (nearReading.getSource().equals(lastSource)) {
                        numReadingsLastSource++;

                        if (numReadingsLastSource >= Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH) {
                            readingsToRemove.add(nearReading);
                        }
                    }
                }

                nearReadings.removeAll(readingsToRemove);

                locatedFingerprints.add(
                        (RssiFingerprintLocated3D<RadioSource, RssiReading<RadioSource>>)nearFingerprint);
            }


            //create estimator
            NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                    new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                            locatedFingerprints, fingerprint,
                            position,
                            sources,
                            this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);
            estimator.setMinMaxNearestFingerprints(minNearestFingerprints,-1);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            Matrix estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            List<RadioSourceLocated<Point3D>> estimatedLocatedSources =
                    estimator.getEstimatedLocatedSources();
            Matrix estimatedCovariance = estimator.getCovariance();

            if (estimatedPosition == null) {
                continue;
            }

            assertNotNull(estimatedPosition);

            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertEquals(estimatedLocatedSources.size(), numSources - 1);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            double positionError = estimatedPosition.distanceTo(position);

            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(positionError <= ABSOLUTE_ERROR);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                double positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            double avgSourcePositionErrorOneTime = 0.0;
            double avgSourceAccuracyOneTime = 0.0;
            int numValidSources = 0;
            for (RadioSourceLocated<Point3D> estimatedLocatedSource : estimatedLocatedSources) {
                Point3D estimatedSourcePosition = estimatedLocatedSource.getPosition();
                Matrix estimatedSourcePositionCovariance =
                        estimatedLocatedSource.getPositionCovariance();

                int index = sources.indexOf(estimatedLocatedSource);
                RadioSourceLocated<Point3D> source = sources.get(index);
                Point3D sourcePosition = source.getPosition();
                double sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                if (sourcePositionError > ABSOLUTE_ERROR) {
                    continue;
                }
                assertTrue(sourcePositionError <= ABSOLUTE_ERROR);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    double sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        double percentage = (double)numValid / (double)TIMES;

        LOGGER.log(Level.INFO, "Valid percentage: {0}%",
                percentage * 100.0);
        LOGGER.log(Level.INFO, "Avg. position error: {0} m",
                avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m",
                avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m",
                avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m",
                avgSourceAccuracy);

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateMultipleRadioSourcesWithoutInitialValues()
            throws LockedException, FingerprintEstimationException, NotReadyException {

        Accuracy3D accuracy = new Accuracy3D();

        double avgPositionError = 0.0;
        double avgSourcePositionError = 0.0;
        double avgPositionAccuracy = 0.0;
        double avgSourceAccuracy = 0.0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            //build sources
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for(int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                                transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            //build located fingerprints
            int numFingerprints = Math.max(
                    randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS),
                    2 * (Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1));
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
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
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

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //create estimator
            NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                    new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                            locatedFingerprints, fingerprint, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            Matrix estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            List<RadioSourceLocated<Point3D>> estimatedLocatedSources =
                    estimator.getEstimatedLocatedSources();
            Matrix estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertEquals(estimatedLocatedSources.size(), numSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            double positionError = estimatedPosition.distanceTo(position);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                double positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            double avgSourcePositionErrorOneTime = 0.0;
            double avgSourceAccuracyOneTime = 0.0;
            int numValidSources = 0;
            for (RadioSourceLocated<Point3D> estimatedLocatedSource : estimatedLocatedSources) {
                Point3D estimatedSourcePosition = estimatedLocatedSource.getPosition();
                Matrix estimatedSourcePositionCovariance =
                        estimatedLocatedSource.getPositionCovariance();

                int index = sources.indexOf(estimatedLocatedSource);
                RadioSourceLocated<Point3D> source = sources.get(index);
                Point3D sourcePosition = source.getPosition();
                double sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    double sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;

            break;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        LOGGER.log(Level.INFO, "Avg. position error: {0} m",
                avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m",
                avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m",
                avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m",
                avgSourceAccuracy);

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateMultipleRadioSourcesWithExactInitialValues()
            throws LockedException, FingerprintEstimationException, NotReadyException {

        Accuracy3D accuracy = new Accuracy3D();

        double avgPositionError = 0.0;
        double avgSourcePositionError = 0.0;
        double avgPositionAccuracy = 0.0;
        double avgSourceAccuracy = 0.0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            //build sources
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            for(int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                                transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            //build located fingerprints
            int numFingerprints = Math.max(
                    randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS),
                    2 * (Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1));
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
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
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

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //create estimator
            NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                    new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                            locatedFingerprints, fingerprint,
                            position, sources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            Matrix estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            List<RadioSourceLocated<Point3D>> estimatedLocatedSources =
                    estimator.getEstimatedLocatedSources();
            Matrix estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertEquals(estimatedLocatedSources.size(), numSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            double positionError = estimatedPosition.distanceTo(position);

            if (positionError > ABSOLUTE_ERROR) {
                continue;
            }
            assertTrue(positionError <= ABSOLUTE_ERROR);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                double positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            double avgSourcePositionErrorOneTime = 0.0;
            double avgSourceAccuracyOneTime = 0.0;
            int numValidSources = 0;
            for (RadioSourceLocated<Point3D> estimatedLocatedSource : estimatedLocatedSources) {
                Point3D estimatedSourcePosition = estimatedLocatedSource.getPosition();
                Matrix estimatedSourcePositionCovariance =
                        estimatedLocatedSource.getPositionCovariance();

                int index = sources.indexOf(estimatedLocatedSource);
                RadioSourceLocated<Point3D> source = sources.get(index);
                Point3D sourcePosition = source.getPosition();
                double sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                if (sourcePositionError > ABSOLUTE_ERROR) {
                    continue;
                }
                assertTrue(sourcePositionError <= ABSOLUTE_ERROR);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    double sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;

            break;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        LOGGER.log(Level.INFO, "Avg. position error: {0} m",
                avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m",
                avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m",
                avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m",
                avgSourceAccuracy);

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateMultipleRadioSourcesWithApproximateInitialValues()
            throws LockedException, FingerprintEstimationException, NotReadyException {

        Accuracy3D accuracy = new Accuracy3D();

        double avgPositionError = 0.0;
        double avgSourcePositionError = 0.0;
        double avgPositionAccuracy = 0.0;
        double avgSourceAccuracy = 0.0;
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());
            GaussianRandomizer positionErrorRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, INITIAL_POSITION_ERROR_STD);

            //build sources
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
            List<RadioSourceLocated<Point3D>> sources = new ArrayList<>();
            List<RadioSourceLocated<Point3D>> initialSources = new ArrayList<>();
            for(int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double z = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint3D position = new InhomogeneousPoint3D(x, y, z);

                InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                        x + positionErrorRandomizer.nextDouble(),
                        y + positionErrorRandomizer.nextDouble(),
                        z + positionErrorRandomizer.nextDouble());

                WifiAccessPointWithPowerAndLocated3D accessPoint =
                        new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                                transmittedPowerdBm, position);
                sources.add(accessPoint);

                WifiAccessPointWithPowerAndLocated3D initialAccessPoint =
                        new WifiAccessPointWithPowerAndLocated3D("bssid" + i, FREQUENCY,
                                transmittedPowerdBm, initialPosition);
                initialSources.add(initialAccessPoint);
            }

            //build located fingerprints
            int numFingerprints = Math.max(
                    randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS),
                    2 * (Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH + 1));
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
                            distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
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

            InhomogeneousPoint3D initialPosition = new InhomogeneousPoint3D(
                    x + positionErrorRandomizer.nextDouble(),
                    y + positionErrorRandomizer.nextDouble(),
                    z + positionErrorRandomizer.nextDouble());

            List<RssiReading<RadioSource>> readings = new ArrayList<>();
            for (RadioSourceLocated<Point3D> source : sources) {
                double distance = source.getPosition().distanceTo(position);
                double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated3D) source).
                        getTransmittedPower();

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                        distance, NonLinearFingerprintPositionAndRadioSourceEstimator.DEFAULT_PATH_LOSS_EXPONENT));
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //create estimator
            NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator =
                    new NonLinearFingerprintPositionAndRadioSourceEstimator3D(
                            locatedFingerprints, fingerprint,
                            initialPosition, initialSources, this);
            estimator.setUseNoMeanNearestFingerprintFinder(true);

            reset();

            //check is ready
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertNull(estimator.getEstimatedPosition());
            assertNull(estimator.getEstimatedLocatedSources());
            assertNull(estimator.getNearestFingerprints());
            assertEquals(estimateStart, 0);
            assertEquals(estimateEnd, 0);

            //estimate
            estimator.estimate();

            Point3D estimatedPosition = estimator.getEstimatedPosition();
            Matrix estimatedPositionCovariance = estimator.getEstimatedPositionCovariance();
            List<RadioSourceLocated<Point3D>> estimatedLocatedSources =
                    estimator.getEstimatedLocatedSources();
            Matrix estimatedCovariance = estimator.getCovariance();

            assertNotNull(estimatedPosition);
            assertNotNull(estimatedPositionCovariance);
            assertNotNull(estimatedLocatedSources);
            assertEquals(estimatedLocatedSources.size(), numSources);
            assertNotNull(estimatedCovariance);
            assertFalse(estimator.isLocked());
            assertTrue(estimator.isReady());
            assertEquals(estimateStart, 1);
            assertEquals(estimateEnd, 1);

            double positionError = estimatedPosition.distanceTo(position);

            try {
                accuracy.setCovarianceMatrix(estimatedPositionCovariance);

                double positionAccuracy = accuracy.getAverageAccuracyMeters();
                avgPositionAccuracy += positionAccuracy;
            } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                continue;
            }

            avgPositionError += positionError;

            double avgSourcePositionErrorOneTime = 0.0;
            double avgSourceAccuracyOneTime = 0.0;
            int numValidSources = 0;
            for (RadioSourceLocated<Point3D> estimatedLocatedSource : estimatedLocatedSources) {
                Point3D estimatedSourcePosition = estimatedLocatedSource.getPosition();
                Matrix estimatedSourcePositionCovariance =
                        estimatedLocatedSource.getPositionCovariance();

                int index = sources.indexOf(estimatedLocatedSource);
                RadioSourceLocated<Point3D> source = sources.get(index);
                Point3D sourcePosition = source.getPosition();
                double sourcePositionError = estimatedSourcePosition.distanceTo(sourcePosition);

                try {
                    accuracy.setCovarianceMatrix(estimatedSourcePositionCovariance);

                    double sourcePositionAccuracy = accuracy.getAverageAccuracyMeters();
                    avgSourceAccuracyOneTime += sourcePositionAccuracy;
                } catch (NonSymmetricPositiveDefiniteMatrixException ignore) {
                    continue;
                }

                avgSourcePositionErrorOneTime += sourcePositionError;

                numValidSources++;
            }

            avgSourcePositionErrorOneTime /= numValidSources;
            avgSourceAccuracyOneTime /= numValidSources;

            avgSourcePositionError += avgSourcePositionErrorOneTime;
            avgSourceAccuracy += avgSourceAccuracyOneTime;

            numValid++;

            break;
        }

        avgPositionError /= numValid;
        avgSourcePositionError /= numValid;
        avgPositionAccuracy /= numValid;
        avgSourceAccuracy /= numValid;

        LOGGER.log(Level.INFO, "Avg. position error: {0} m",
                avgPositionError);
        LOGGER.log(Level.INFO, "Avg. position accuracy: {0} m",
                avgPositionAccuracy);

        LOGGER.log(Level.INFO, "Avg. source position error: {0} m",
                avgSourcePositionError);
        LOGGER.log(Level.INFO, "Avg. source accuracy: {0} m",
                avgSourceAccuracy);

        assertTrue(numValid > 0);
    }

    @Override
    public void onEstimateStart(FingerprintPositionAndRadioSourceEstimator<Point3D> estimator) {
        estimateStart++;
        checkLocked((NonLinearFingerprintPositionAndRadioSourceEstimator3D)estimator);
    }

    @Override
    public void onEstimateEnd(FingerprintPositionAndRadioSourceEstimator<Point3D> estimator) {
        estimateEnd++;
        checkLocked((NonLinearFingerprintPositionAndRadioSourceEstimator3D)estimator);
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

    private void checkLocked(NonLinearFingerprintPositionAndRadioSourceEstimator3D estimator) {
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
            estimator.estimate();
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) {
        } catch (Exception e) {
            fail("LockedException expected but not thrown");
        }
    }
}
