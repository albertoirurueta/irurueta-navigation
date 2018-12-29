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

import com.irurueta.geometry.InhomogeneousPoint2D;
import com.irurueta.geometry.Point2D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.indoor.*;
import com.irurueta.numerical.EvaluationException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

public class NonLinearSourcedRssiPositionEstimator2DTest
        implements SourcedRssiPositionEstimatorListener<Point2D> {

    private static final Logger LOGGER = Logger.getLogger(
            NonLinearSourcedRssiPositionEstimator2DTest.class.getName());

    private static final double FREQUENCY = 2.4e9; //(Hz)

    private static final int MIN_SOURCES = 3;
    private static final int MAX_SOURCES = 10;

    private static final double MIN_POS = -50.0;
    private static final double MAX_POS = 50.0;

    private static final double MIN_RSSI = -100;
    private static final double MAX_RSSI = -50;

    private static final int MIN_FINGERPRINTS = 100;
    private static final int MAX_FINGERPRINTS = 1000;

    private static final double SPEED_OF_LIGHT = 299792458.0;

    private static final int TIMES = 100;

    public NonLinearSourcedRssiPositionEstimator2DTest() { }

    @BeforeClass
    public static void setUpClass() { }

    @AfterClass
    public static void tearDownClass() { }

    @Before
    public void setUp() { }

    @After
    public void tearDown() { }

    @Test
    public void testCreate() {
        //test create empty

        //first order
        NonLinearSourcedRssiPositionEstimator2D estimator =
                NonLinearSourcedRssiPositionEstimator2D.create(
                        NonLinearSourcedRssiPositionEstimatorType.FIRST_ORDER);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.FIRST_ORDER);
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        //second order
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(
                NonLinearSourcedRssiPositionEstimatorType.SECOND_ORDER);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.SECOND_ORDER);
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        //third order
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(
                NonLinearSourcedRssiPositionEstimatorType.THIRD_ORDER);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.THIRD_ORDER);
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        //default type
        estimator = NonLinearSourcedRssiPositionEstimator2D.create();

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.FIRST_ORDER);
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());


        //test create with listener

        //first order
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(this,
                NonLinearSourcedRssiPositionEstimatorType.FIRST_ORDER);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.FIRST_ORDER);
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertSame(estimator.getListener(), this);

        //second order
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(this,
                NonLinearSourcedRssiPositionEstimatorType.SECOND_ORDER);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.SECOND_ORDER);
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertSame(estimator.getListener(), this);

        //third order
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(this,
                NonLinearSourcedRssiPositionEstimatorType.THIRD_ORDER);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.THIRD_ORDER);
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertSame(estimator.getListener(), this);

        //default type
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(this);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.FIRST_ORDER);
        assertNull(estimator.getLocatedFingerprints());
        assertNull(estimator.getFingerprint());
        assertNull(estimator.getSources());
        assertNull(estimator.getInitialPosition());
        assertSame(estimator.getListener(), this);


        //test create with located fingerprints, fingerprint and sources
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


        RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                new RssiFingerprint<>(readings);


        List<RadioSourceLocated<Point2D>> sources = new ArrayList<>();
        for (int i = 0; i < Point2D.POINT2D_INHOMOGENEOUS_COORDINATES_LENGTH; i++) {
            WifiAccessPointLocated2D source = new WifiAccessPointLocated2D("bssid" + 1,
                    FREQUENCY, Point2D.create());
            sources.add(source);
        }

        //first order
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(locatedFingerprints,
                fingerprint, sources, NonLinearSourcedRssiPositionEstimatorType.FIRST_ORDER);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.FIRST_ORDER);
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        //second order
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(locatedFingerprints,
                fingerprint, sources, NonLinearSourcedRssiPositionEstimatorType.SECOND_ORDER);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.SECOND_ORDER);
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        //third order
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(locatedFingerprints,
                fingerprint, sources, NonLinearSourcedRssiPositionEstimatorType.THIRD_ORDER);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.THIRD_ORDER);
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());

        //default type
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(locatedFingerprints,
                fingerprint, sources);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.FIRST_ORDER);
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getInitialPosition());
        assertNull(estimator.getListener());


        //test create with located fingerprints, fingerprint, sources and listener

        //first order
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(locatedFingerprints,
                fingerprint, sources, this,
                NonLinearSourcedRssiPositionEstimatorType.FIRST_ORDER);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.FIRST_ORDER);
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getInitialPosition());
        assertSame(estimator.getListener(), this);

        //second order
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(locatedFingerprints,
                fingerprint, sources, this,
                NonLinearSourcedRssiPositionEstimatorType.SECOND_ORDER);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.SECOND_ORDER);
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getInitialPosition());
        assertSame(estimator.getListener(), this);

        //third order
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(locatedFingerprints,
                fingerprint, sources, this,
                NonLinearSourcedRssiPositionEstimatorType.THIRD_ORDER);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.THIRD_ORDER);
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getInitialPosition());
        assertSame(estimator.getListener(), this);

        //default type
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(locatedFingerprints,
                fingerprint, sources, this);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.FIRST_ORDER);
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getSources(), sources);
        assertNull(estimator.getInitialPosition());
        assertSame(estimator.getListener(), this);


        //test create with located fingerprints, fingerprint, sources and initial position
        Point2D initialPosition = Point2D.create();

        //first order
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(locatedFingerprints,
                fingerprint, sources, initialPosition,
                NonLinearSourcedRssiPositionEstimatorType.FIRST_ORDER);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.FIRST_ORDER);
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertNull(estimator.getListener());

        //second order
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(locatedFingerprints,
                fingerprint, sources, initialPosition,
                NonLinearSourcedRssiPositionEstimatorType.SECOND_ORDER);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.SECOND_ORDER);
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertNull(estimator.getListener());

        //third order
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(locatedFingerprints,
                fingerprint, sources, initialPosition,
                NonLinearSourcedRssiPositionEstimatorType.THIRD_ORDER);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.THIRD_ORDER);
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertNull(estimator.getListener());

        //default type
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(locatedFingerprints,
                fingerprint, sources, initialPosition);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.FIRST_ORDER);
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertNull(estimator.getListener());


        //test create with located fingerprints, fingerprint, sources, initial position
        //and listener

        //first order
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(locatedFingerprints,
                fingerprint, sources, initialPosition, this,
                NonLinearSourcedRssiPositionEstimatorType.FIRST_ORDER);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.FIRST_ORDER);
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);

        //second order
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(locatedFingerprints,
                fingerprint, sources, initialPosition, this,
                NonLinearSourcedRssiPositionEstimatorType.SECOND_ORDER);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.SECOND_ORDER);
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);

        //third order
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(locatedFingerprints,
                fingerprint, sources, initialPosition, this,
                NonLinearSourcedRssiPositionEstimatorType.THIRD_ORDER);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.THIRD_ORDER);
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);

        //default type
        estimator = NonLinearSourcedRssiPositionEstimator2D.create(locatedFingerprints,
                fingerprint, sources, initialPosition, this);

        //check
        assertEquals(estimator.getType(),
                NonLinearSourcedRssiPositionEstimatorType.FIRST_ORDER);
        assertSame(estimator.getLocatedFingerprints(), locatedFingerprints);
        assertSame(estimator.getFingerprint(), fingerprint);
        assertSame(estimator.getSources(), sources);
        assertSame(estimator.getInitialPosition(), initialPosition);
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testEstimateOrderComparison() throws EvaluationException,
            LockedException, NotReadyException, PositionEstimationException {
        double avgRssiErrorFirstOrder = 0.0;
        double avgRssiErrorSecondOrder = 0.0;
        double avgRssiErrorThirdOrder = 0.0;

        double avgPositionErrorFirstOrder = 0.0;
        double avgPositionErrorSecondOrder = 0.0;
        double avgPositionErrorThirdOrder = 0.0;
        for (int t = 0; t < TIMES; t++) {
            UniformRandomizer randomizer = new UniformRandomizer(new Random());

            //build sources
            int numSources = randomizer.nextInt(MIN_SOURCES, MAX_SOURCES);
            List<RadioSourceLocated<Point2D>> sources = new ArrayList<>();
            for (int i = 0; i < numSources; i++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                double transmittedPowerdBm = randomizer.nextDouble(MIN_RSSI, MAX_RSSI);
                InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

                WifiAccessPointWithPowerAndLocated2D accessPoint =
                        new WifiAccessPointWithPowerAndLocated2D(
                                "bssid" + i, FREQUENCY, transmittedPowerdBm, position);
                sources.add(accessPoint);
            }

            //build located fingerprints
            int numFingerprints = randomizer.nextInt(MIN_FINGERPRINTS, MAX_FINGERPRINTS);
            List<RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>>> locatedFingerprints =
                    new ArrayList<>();
            for (int j = 0; j < numFingerprints; j++) {
                double x = randomizer.nextDouble(MIN_POS, MAX_POS);
                double y = randomizer.nextDouble(MIN_POS, MAX_POS);
                InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

                List<RssiReading<RadioSource>> readings = new ArrayList<>();
                for (RadioSourceLocated<Point2D> source : sources) {
                    double distance = source.getPosition().distanceTo(position);
                    double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated2D) source).
                            getTransmittedPower();

                    double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm),
                            distance));
                    RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                            receivedRssi);
                    readings.add(reading);
                }

                RssiFingerprintLocated2D<RadioSource, RssiReading<RadioSource>> locatedFingerprint =
                        new RssiFingerprintLocated2D<>(readings, position);
                locatedFingerprints.add(locatedFingerprint);
            }

            //build non-located fingerprint
            double x = randomizer.nextDouble(MIN_POS, MAX_POS);
            double y = randomizer.nextDouble(MIN_POS, MAX_POS);
            InhomogeneousPoint2D position = new InhomogeneousPoint2D(x, y);

            List<RssiReading<RadioSource>> readings = new ArrayList<>();
            for (RadioSourceLocated<Point2D> source : sources) {
                double distance = source.getPosition().distanceTo(position);
                double transmittedPowerdBm = ((WifiAccessPointWithPowerAndLocated2D) source).
                        getTransmittedPower();

                double receivedRssi = Utils.powerTodBm(receivedPower(Utils.dBmToPower(transmittedPowerdBm), distance
                ));
                RssiReading<RadioSource> reading = new RssiReading<>((RadioSource) source,
                        receivedRssi);
                readings.add(reading);
            }

            RssiFingerprint<RadioSource, RssiReading<RadioSource>> fingerprint =
                    new RssiFingerprint<>(readings);

            //find closest fingerprint based on RSSI without mean
            RadioSourceNoMeanKNearestFinder<Point2D, RadioSource> fingerprintFinder =
                    new RadioSourceNoMeanKNearestFinder<>(locatedFingerprints);

            RssiFingerprintLocated<RadioSource, RssiReading<RadioSource>, Point2D> nearestFingerprint =
                    fingerprintFinder.findNearestTo(fingerprint);
            Point2D nearestFingerprintPosition = nearestFingerprint.getPosition();


            NonLinearSourcedRssiPositionEstimator2D firstOrderEstimator =
                    NonLinearSourcedRssiPositionEstimator2D.create(
                            locatedFingerprints, fingerprint, sources, this,
                            NonLinearSourcedRssiPositionEstimatorType.FIRST_ORDER);
            NonLinearSourcedRssiPositionEstimator2D secondOrderEstimator =
                    NonLinearSourcedRssiPositionEstimator2D.create(
                            locatedFingerprints, fingerprint, sources, this,
                            NonLinearSourcedRssiPositionEstimatorType.SECOND_ORDER);
            NonLinearSourcedRssiPositionEstimator2D thirdOrderEstimator =
                    NonLinearSourcedRssiPositionEstimator2D.create(
                            locatedFingerprints, fingerprint, sources, this,
                            NonLinearSourcedRssiPositionEstimatorType.THIRD_ORDER);

            //compute average RSSI error for each order
            double[] params = new double[2];
            params[0] = x;
            params[1] = y;

            double[] point = new double[6];
            double[] derivatives = new double[2];
            for (int i = 0; i < numSources; i++) {
                RadioSourceLocated<Point2D> source = sources.get(i);
                RssiReading<RadioSource> reading = readings.get(i);

                Point2D sourcePosition = source.getPosition();

                double readingRssi = reading.getRssi();

                point[0] = readingRssi;

                point[1] = nearestFingerprintPosition.getInhomX();
                point[2] = nearestFingerprintPosition.getInhomY();

                point[3] = sourcePosition.getInhomX();
                point[4] = sourcePosition.getInhomY();

                point[5] = LinearSourcedRssiPositionEstimator2D.DEFAULT_PATH_LOSS_EXPONENT;

                double estimatedRssiFirstOrder = firstOrderEstimator.evaluate(
                        0, point, params, derivatives);
                double estimatedRssiSecondOrder = secondOrderEstimator.evaluate(
                        0, point, params, derivatives);
                double estimatedRssiThirdOrder = thirdOrderEstimator.evaluate(
                        0, point, params, derivatives);

                double rssiErrorFirstOrder = Math.abs(
                        estimatedRssiFirstOrder - readingRssi);
                double rssiErrorSecondOrder = Math.abs(
                        estimatedRssiSecondOrder - readingRssi);
                double rssiErrorThirdOrder = Math.abs(
                        estimatedRssiThirdOrder - readingRssi);

                avgRssiErrorFirstOrder += rssiErrorFirstOrder / TIMES;
                avgRssiErrorSecondOrder += rssiErrorSecondOrder / TIMES;
                avgRssiErrorThirdOrder += rssiErrorThirdOrder / TIMES;

                firstOrderEstimator.estimate();
                secondOrderEstimator.estimate();
                thirdOrderEstimator.estimate();

                Point2D estimatedPositionFirstOrder = firstOrderEstimator.
                        getEstimatedPosition();
                Point2D estimatedPositionSecondOrder = secondOrderEstimator.
                        getEstimatedPosition();
                Point2D estimatedPositionThirdOrder = thirdOrderEstimator.
                        getEstimatedPosition();

                double positionErrorFirstOrder = estimatedPositionFirstOrder.
                        distanceTo(position);
                double positionErrorSecondOrder = estimatedPositionSecondOrder.
                        distanceTo(position);
                double positionErrorThirdOrder = estimatedPositionThirdOrder.
                        distanceTo(position);

                avgPositionErrorFirstOrder += positionErrorFirstOrder / TIMES;
                avgPositionErrorSecondOrder += positionErrorSecondOrder / TIMES;
                avgPositionErrorThirdOrder += positionErrorThirdOrder / TIMES;
            }
        }

        LOGGER.log(Level.INFO, "Avg rssi error 1st order: {0} dBm",
                avgRssiErrorFirstOrder);
        LOGGER.log(Level.INFO, "Avg rssi error 2nd order: {0} dBm",
                avgRssiErrorSecondOrder);
        LOGGER.log(Level.INFO, "Avg rssi error 3rd order: {0} dBm",
                avgRssiErrorThirdOrder);

        LOGGER.log(Level.INFO, "Avg position error 1st order: {0} m",
                avgPositionErrorFirstOrder);
        LOGGER.log(Level.INFO, "Avg position error 2nd order: {0} m",
                avgPositionErrorSecondOrder);
        LOGGER.log(Level.INFO, "Avg position error 3rd order: {0} m",
                avgPositionErrorThirdOrder);

        assertTrue(avgPositionErrorFirstOrder >= avgPositionErrorSecondOrder);
        assertTrue(avgPositionErrorFirstOrder >= avgPositionErrorThirdOrder);
//        assertTrue(avgPositionErrorSecondOrder >= avgPositionErrorThirdOrder);
    }

    @Override
    public void onEstimateStart(SourcedRssiPositionEstimator<Point2D> estimator) {
        checkLocked((NonLinearSourcedRssiPositionEstimator2D)estimator);
    }

    @Override
    public void onEstimateEnd(SourcedRssiPositionEstimator<Point2D> estimator) {
        checkLocked((NonLinearSourcedRssiPositionEstimator2D)estimator);
    }

    private double receivedPower(double equivalentTransmittedPower,
                                 double distance) {
        //Pr = Pt*Gt*Gr*lambda^2/(4*pi*d)^2,    where Pr is the received power
        // lambda = c/f, where lambda is wavelength,
        // Pte = Pt*Gt*Gr, is the equivalent transmitted power, Gt is the transmitted Gain and Gr is the received Gain
        //Pr = Pte*c^2/((4*pi*f)^2 * d^2)
        double k = Math.pow(SPEED_OF_LIGHT / (4.0 * Math.PI * FREQUENCY),
                BaseRssiPositionEstimator.DEFAULT_PATH_LOSS_EXPONENT);
        return equivalentTransmittedPower * k /
                Math.pow(distance, BaseRssiPositionEstimator.DEFAULT_PATH_LOSS_EXPONENT);
    }

    private void checkLocked(NonLinearSourcedRssiPositionEstimator2D estimator) {
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
            estimator.setInitialPosition(null);
            fail("LockedException expected but not thrown");
        } catch (LockedException ignore) { }
        try {
            estimator.setFallbackRssiStandardDeviation(1.0);
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
