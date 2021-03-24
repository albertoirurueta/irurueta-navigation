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
package com.irurueta.navigation.gnss;

import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Before;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class GNSSLeastSquaresPositionAndVelocityEstimatorTest implements
        GNSSLeastSquaresPositionAndVelocityEstimatorListener {

    private static final double MIN_CONVERGENCE = 1e-6;
    private static final double MAX_CONVERGENCE = 1.0;

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final double MIN_MASK_ANGLE_DEGREES = 15.0;
    private static final double MAX_MASK_ANGLE_DEGREES = 20.0;

    private static final int MIN_SATELLITES = 4;
    private static final int MAX_SATELLITES = 10;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_USER_HEIGHT = -50.0;
    private static final double MAX_USER_HEIGHT = 50.0;

    private static final double MIN_SAT_HEIGHT = 150000.0;
    private static final double MAX_SAT_HEIGHT = 800000.0;

    private static final double MIN_USER_VELOCITY_VALUE = -2.0;
    private static final double MAX_USER_VELOCITY_VALUE = 2.0;

    private static final double MIN_SAT_VELOCITY_VALUE = -3500.0;
    private static final double MAX_SAT_VELOCITY_VALUE = 3500.0;

    private static final double MIN_TIME = 0.0;
    private static final double MAX_TIME = 1.0;

    private static final int MIN_NUM_SAT = 4;
    private static final int MAX_NUM_SAT = 10;

    private static final double POSITION_ERROR = 5e-1;
    private static final double VELOCITY_ERROR = 5e-2;

    private static final double MIN_PRIOR_POSITION_ERROR = -500.0;
    private static final double MAX_PRIOR_POSITION_ERROR = 500.0;

    private static final double MIN_PRIOR_VELOCITY_ERROR = -1.0;
    private static final double MAX_PRIOR_VELOCITy_ERROR = 1.0;

    private static final int TIMES = 100;

    private int mEstimateStart;
    private int mEstimateEnd;

    @Before
    public void setUp() {
        reset();
    }

    @Test
    public void testConstructor() {
        // test empty constructor
        GNSSLeastSquaresPositionAndVelocityEstimator estimator =
                new GNSSLeastSquaresPositionAndVelocityEstimator();

        // check default values
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getPriorPositionAndVelocity());
        assertNull(estimator.getListener());
        assertEquals(estimator.getConvergenceThreshold(),
                GNSSLeastSquaresPositionAndVelocityEstimator.CONVERGENCE_THRESHOLD,
                0.0);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());


        // test constructor with measurements
        final GNSSMeasurement measurement1 = new GNSSMeasurement();
        final GNSSMeasurement measurement2 = new GNSSMeasurement();
        final GNSSMeasurement measurement3 = new GNSSMeasurement();
        final GNSSMeasurement measurement4 = new GNSSMeasurement();
        final List<GNSSMeasurement> measurements = new ArrayList<>();
        measurements.add(measurement1);
        measurements.add(measurement2);
        measurements.add(measurement3);
        measurements.add(measurement4);

        estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(measurements);

        // check default values
        assertSame(estimator.getMeasurements(), measurements);
        assertNull(estimator.getPriorPositionAndVelocity());
        assertNull(estimator.getListener());
        assertEquals(estimator.getConvergenceThreshold(),
                GNSSLeastSquaresPositionAndVelocityEstimator.CONVERGENCE_THRESHOLD,
                0.0);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(
                    Collections.<GNSSMeasurement>emptyList());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor with measurements and prior position and velocity
        final ECEFPositionAndVelocity priorPositionAndVelocity =
                new ECEFPositionAndVelocity();

        estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(measurements,
                priorPositionAndVelocity);

        // check default values
        assertSame(estimator.getMeasurements(), measurements);
        assertSame(estimator.getPriorPositionAndVelocity(), priorPositionAndVelocity);
        assertNull(estimator.getListener());
        assertEquals(estimator.getConvergenceThreshold(),
                GNSSLeastSquaresPositionAndVelocityEstimator.CONVERGENCE_THRESHOLD,
                0.0);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(
                    Collections.<GNSSMeasurement>emptyList(),
                    priorPositionAndVelocity);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor with measurements and prior result
        final GNSSEstimation priorEstimation = new GNSSEstimation();
        estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(measurements,
                priorEstimation);

        // check default values
        assertSame(estimator.getMeasurements(), measurements);
        assertNotNull(estimator.getPriorPositionAndVelocity());
        assertNull(estimator.getListener());
        assertEquals(estimator.getConvergenceThreshold(),
                GNSSLeastSquaresPositionAndVelocityEstimator.CONVERGENCE_THRESHOLD,
                0.0);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(
                    Collections.<GNSSMeasurement>emptyList(),
                    priorEstimation);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor with listener
        estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(this);

        // check default values
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getPriorPositionAndVelocity());
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getConvergenceThreshold(),
                GNSSLeastSquaresPositionAndVelocityEstimator.CONVERGENCE_THRESHOLD,
                0.0);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());


        // test constructor with measurements and listener
        estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(
                measurements, this);

        // check default values
        assertSame(estimator.getMeasurements(), measurements);
        assertNull(estimator.getPriorPositionAndVelocity());
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getConvergenceThreshold(),
                GNSSLeastSquaresPositionAndVelocityEstimator.CONVERGENCE_THRESHOLD,
                0.0);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(
                    Collections.<GNSSMeasurement>emptyList(), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor with measurements, prior position and velocity and listener
        estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(measurements,
                priorPositionAndVelocity, this);

        // check default values
        assertSame(estimator.getMeasurements(), measurements);
        assertSame(estimator.getPriorPositionAndVelocity(), priorPositionAndVelocity);
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getConvergenceThreshold(),
                GNSSLeastSquaresPositionAndVelocityEstimator.CONVERGENCE_THRESHOLD,
                0.0);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(
                    Collections.<GNSSMeasurement>emptyList(),
                    priorPositionAndVelocity, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor with measurements and prior result
        estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(measurements,
                priorEstimation, this);

        // check default values
        assertSame(estimator.getMeasurements(), measurements);
        assertNotNull(estimator.getPriorPositionAndVelocity());
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getConvergenceThreshold(),
                GNSSLeastSquaresPositionAndVelocityEstimator.CONVERGENCE_THRESHOLD,
                0.0);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(
                    Collections.<GNSSMeasurement>emptyList(),
                    priorEstimation, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetMeasurements() throws LockedException {
        final GNSSLeastSquaresPositionAndVelocityEstimator estimator =
                new GNSSLeastSquaresPositionAndVelocityEstimator();

        // check default value
        assertNull(estimator.getMeasurements());

        // set new value
        final GNSSMeasurement measurement1 = new GNSSMeasurement();
        final GNSSMeasurement measurement2 = new GNSSMeasurement();
        final GNSSMeasurement measurement3 = new GNSSMeasurement();
        final GNSSMeasurement measurement4 = new GNSSMeasurement();
        final List<GNSSMeasurement> measurements = new ArrayList<>();
        measurements.add(measurement1);
        measurements.add(measurement2);
        measurements.add(measurement3);
        measurements.add(measurement4);

        estimator.setMeasurements(measurements);

        // check
        assertSame(estimator.getMeasurements(), measurements);

        // Force IllegalArgumentException
        try {
            estimator.setMeasurements(Collections.<GNSSMeasurement>emptyList());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetPriorPositionAndVelocity() throws LockedException {
        final GNSSLeastSquaresPositionAndVelocityEstimator estimator =
                new GNSSLeastSquaresPositionAndVelocityEstimator();

        // check default value
        assertNull(estimator.getPriorPositionAndVelocity());

        // set new value
        final ECEFPositionAndVelocity priorPositionAndVelocity =
                new ECEFPositionAndVelocity();

        estimator.setPriorPositionAndVelocity(priorPositionAndVelocity);

        // check
        assertSame(estimator.getPriorPositionAndVelocity(), priorPositionAndVelocity);
    }

    @Test
    public void testSetPriorPositionAndVelocityFromEstimation() throws LockedException {
        final GNSSLeastSquaresPositionAndVelocityEstimator estimator =
                new GNSSLeastSquaresPositionAndVelocityEstimator();

        // set new value
        final GNSSEstimation priorEstimation = new GNSSEstimation();
        final ECEFPositionAndVelocity priorPositionAndVelocity =
                new ECEFPositionAndVelocity();
        priorPositionAndVelocity.setPositionCoordinates(1.0, 2.0, 3.0);
        priorEstimation.setPositionAndVelocity(priorPositionAndVelocity);

        estimator.setPriorPositionAndVelocityFromEstimation(priorEstimation);

        // check
        assertEquals(estimator.getPriorPositionAndVelocity(), priorPositionAndVelocity);


        estimator.setPriorPositionAndVelocityFromEstimation(null);

        // check
        assertNull(estimator.getPriorPositionAndVelocity());
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final GNSSLeastSquaresPositionAndVelocityEstimator estimator =
                new GNSSLeastSquaresPositionAndVelocityEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testGetSetConvergenceThreshold() throws LockedException {
        final GNSSLeastSquaresPositionAndVelocityEstimator estimator =
                new GNSSLeastSquaresPositionAndVelocityEstimator();

        // check default value
        assertEquals(estimator.getConvergenceThreshold(),
                GNSSLeastSquaresPositionAndVelocityEstimator.CONVERGENCE_THRESHOLD,
                0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double threshold = randomizer.nextDouble(
                MIN_CONVERGENCE, MAX_CONVERGENCE);
        estimator.setConvergenceThreshold(threshold);

        // check
        assertEquals(estimator.getConvergenceThreshold(), threshold, 0.0);
    }

    @Test
    public void testIsValidMeasurements() {
        assertFalse(GNSSLeastSquaresPositionAndVelocityEstimator.isValidMeasurements(
                Collections.<GNSSMeasurement>emptyList()));

        final GNSSMeasurement measurement1 = new GNSSMeasurement();
        final GNSSMeasurement measurement2 = new GNSSMeasurement();
        final GNSSMeasurement measurement3 = new GNSSMeasurement();
        final GNSSMeasurement measurement4 = new GNSSMeasurement();
        final List<GNSSMeasurement> measurements = new ArrayList<>();
        measurements.add(measurement1);
        measurements.add(measurement2);
        measurements.add(measurement3);
        measurements.add(measurement4);

        assertTrue(GNSSLeastSquaresPositionAndVelocityEstimator.isValidMeasurements(
                measurements));
    }

    @Test
    public void testEstimateWithoutPriorPositionAndVelocity() throws LockedException, NotReadyException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final int numSatellites = randomizer.nextInt(MIN_NUM_SAT, MAX_NUM_SAT);
            final double timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

            final double userLatitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double userLongitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
            final NEDPosition nedUserPosition =
                    new NEDPosition(userLatitude, userLongitude, userHeight);

            final double userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE,
                    MAX_USER_VELOCITY_VALUE);
            final double userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE,
                    MAX_USER_VELOCITY_VALUE);
            final double userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE,
                    MAX_USER_VELOCITY_VALUE);
            final NEDVelocity nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

            final ECEFPosition ecefUserPosition = new ECEFPosition();
            final ECEFVelocity ecefUserVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                    nedUserPosition, nedUserVelocity, ecefUserPosition, ecefUserVelocity);

            final ECEFPositionAndVelocity ecefUserPositionAndVelocity =
                    new ECEFPositionAndVelocity(ecefUserPosition, ecefUserVelocity);

            final GNSSConfig config = generateConfig();
            final double maskAngle = Math.toRadians(config.getMaskAngleDegrees());
            final double delta = maskAngle / 3.0;

            final List<Double> biases = new ArrayList<>();
            final List<ECEFPositionAndVelocity> satellitePositionsAndVelocities =
                    new ArrayList<>();
            for (int n = 0; n < numSatellites; n++) {
                final double satLatitude = randomizer.nextDouble(
                        userLatitude - delta,
                        userLatitude + delta);
                final double satLongitude = randomizer.nextDouble(
                        userLongitude - delta,
                        userLongitude + delta);
                final double satHeight = randomizer.nextDouble(MIN_SAT_HEIGHT,
                        MAX_SAT_HEIGHT);
                final NEDPosition nedSatPosition =
                        new NEDPosition(satLatitude, satLongitude, satHeight);

                final double satVn = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE,
                        MAX_SAT_VELOCITY_VALUE);
                final double satVe = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE,
                        MAX_SAT_VELOCITY_VALUE);
                final double satVd = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE,
                        MAX_SAT_VELOCITY_VALUE);
                final NEDVelocity nedSatVelocity = new NEDVelocity(satVn, satVe, satVd);

                final ECEFPosition ecefSatPosition = new ECEFPosition();
                final ECEFVelocity ecefSatVelocity = new ECEFVelocity();
                NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                        nedSatPosition, nedSatVelocity, ecefSatPosition, ecefSatVelocity);

                final ECEFPositionAndVelocity ecefSatPositionAndVelocity =
                        new ECEFPositionAndVelocity(ecefSatPosition, ecefSatVelocity);

                final double bias = GNSSBiasesGenerator.generateBias(ecefSatPosition,
                        ecefUserPosition, config, random);

                biases.add(bias);
                satellitePositionsAndVelocities.add(ecefSatPositionAndVelocity);
            }

            final Collection<GNSSMeasurement> measurements = GNSSMeasurementsGenerator
                    .generate(timeSeconds, satellitePositionsAndVelocities,
                            ecefUserPositionAndVelocity, biases, config, random);

            if (measurements.size() <
                    GNSSLeastSquaresPositionAndVelocityEstimator.MIN_MEASUREMENTS) {
                continue;
            }

            final GNSSLeastSquaresPositionAndVelocityEstimator estimator =
                    new GNSSLeastSquaresPositionAndVelocityEstimator(measurements, this);

            // check initial values
            reset();
            assertFalse(estimator.isRunning());
            assertEquals(mEstimateStart, 0);
            assertEquals(mEstimateEnd, 0);

            GNSSEstimation estimation;
            try {
                estimation = estimator.estimate();
            } catch (final GNSSException e) {
                continue;
            }

            final ECEFPosition estimatedPosition = estimation.getEcefPosition();
            final ECEFVelocity estimatedVelocity = estimation.getEcefVelocity();

            final double diffX = Math.abs(ecefUserPosition.getX()
                    - estimatedPosition.getX());
            final double diffY = Math.abs(ecefUserPosition.getY()
                    - estimatedPosition.getY());
            final double diffZ = Math.abs(ecefUserPosition.getZ()
                    - estimatedPosition.getZ());
            final double posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition, POSITION_ERROR));

            final double diffVx = Math.abs(ecefUserVelocity.getVx()
                    - estimatedVelocity.getVx());
            final double diffVy = Math.abs(ecefUserVelocity.getVy()
                    - estimatedVelocity.getVy());
            final double diffVz = Math.abs(ecefUserVelocity.getVz()
                    - estimatedVelocity.getVz());
            final double velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity, VELOCITY_ERROR));

            assertEquals(mEstimateStart, 1);
            assertEquals(mEstimateEnd, 1);
            assertFalse(estimator.isRunning());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testEstimateWithPriorPositionAndVelocity() throws LockedException, NotReadyException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final int numSatellites = randomizer.nextInt(MIN_NUM_SAT, MAX_NUM_SAT);
            final double timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

            final double userLatitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double userLongitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
            final NEDPosition nedUserPosition =
                    new NEDPosition(userLatitude, userLongitude, userHeight);

            final double userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE,
                    MAX_USER_VELOCITY_VALUE);
            final double userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE,
                    MAX_USER_VELOCITY_VALUE);
            final double userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE,
                    MAX_USER_VELOCITY_VALUE);
            final NEDVelocity nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

            final ECEFPosition ecefUserPosition = new ECEFPosition();
            final ECEFVelocity ecefUserVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                    nedUserPosition, nedUserVelocity, ecefUserPosition, ecefUserVelocity);

            final ECEFPositionAndVelocity ecefUserPositionAndVelocity =
                    new ECEFPositionAndVelocity(ecefUserPosition, ecefUserVelocity);

            final GNSSConfig config = generateConfig();
            final double maskAngle = Math.toRadians(config.getMaskAngleDegrees());
            final double delta = maskAngle / 3.0;

            final List<Double> biases = new ArrayList<>();
            final List<ECEFPositionAndVelocity> satellitePositionsAndVelocities =
                    new ArrayList<>();
            for (int n = 0; n < numSatellites; n++) {
                final double satLatitude = randomizer.nextDouble(
                        userLatitude - delta,
                        userLatitude + delta);
                final double satLongitude = randomizer.nextDouble(
                        userLongitude - delta,
                        userLongitude + delta);
                final double satHeight = randomizer.nextDouble(MIN_SAT_HEIGHT,
                        MAX_SAT_HEIGHT);
                final NEDPosition nedSatPosition =
                        new NEDPosition(satLatitude, satLongitude, satHeight);

                final double satVn = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE,
                        MAX_SAT_VELOCITY_VALUE);
                final double satVe = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE,
                        MAX_SAT_VELOCITY_VALUE);
                final double satVd = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE,
                        MAX_SAT_VELOCITY_VALUE);
                final NEDVelocity nedSatVelocity = new NEDVelocity(satVn, satVe, satVd);

                final ECEFPosition ecefSatPosition = new ECEFPosition();
                final ECEFVelocity ecefSatVelocity = new ECEFVelocity();
                NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
                        nedSatPosition, nedSatVelocity, ecefSatPosition, ecefSatVelocity);

                final ECEFPositionAndVelocity ecefSatPositionAndVelocity =
                        new ECEFPositionAndVelocity(ecefSatPosition, ecefSatVelocity);

                final double bias = GNSSBiasesGenerator.generateBias(ecefSatPosition,
                        ecefUserPosition, config, random);

                biases.add(bias);
                satellitePositionsAndVelocities.add(ecefSatPositionAndVelocity);
            }

            final Collection<GNSSMeasurement> measurements = GNSSMeasurementsGenerator
                    .generate(timeSeconds, satellitePositionsAndVelocities,
                            ecefUserPositionAndVelocity, biases, config, random);

            if (measurements.size() <
                    GNSSLeastSquaresPositionAndVelocityEstimator.MIN_MEASUREMENTS) {
                continue;
            }

            final double priorX = ecefUserPosition.getX()
                    + randomizer.nextDouble(MIN_PRIOR_POSITION_ERROR, MAX_PRIOR_POSITION_ERROR);
            final double priorY = ecefUserPosition.getY()
                    + randomizer.nextDouble(MIN_PRIOR_POSITION_ERROR, MAX_PRIOR_POSITION_ERROR);
            final double priorZ = ecefUserPosition.getZ()
                    + randomizer.nextDouble(MIN_PRIOR_POSITION_ERROR, MAX_PRIOR_POSITION_ERROR);
            final double priorVx = ecefUserVelocity.getVx()
                    + randomizer.nextDouble(MIN_PRIOR_VELOCITY_ERROR, MAX_PRIOR_VELOCITy_ERROR);
            final double priorVy = ecefUserVelocity.getVy()
                    + randomizer.nextDouble(MIN_PRIOR_VELOCITY_ERROR, MAX_PRIOR_VELOCITy_ERROR);
            final double priorVz = ecefUserVelocity.getVz()
                    + randomizer.nextDouble(MIN_PRIOR_VELOCITY_ERROR, MAX_PRIOR_VELOCITy_ERROR);

            final ECEFPositionAndVelocity priorPositionAndVelocity =
                    new ECEFPositionAndVelocity(priorX, priorY, priorZ,
                            priorVx, priorVy, priorVz);

            final GNSSLeastSquaresPositionAndVelocityEstimator estimator =
                    new GNSSLeastSquaresPositionAndVelocityEstimator(measurements,
                            priorPositionAndVelocity, this);

            // check initial values
            reset();
            assertFalse(estimator.isRunning());
            assertEquals(mEstimateStart, 0);
            assertEquals(mEstimateEnd, 0);

            GNSSEstimation estimation;
            try {
                estimation = estimator.estimate();
            } catch (final GNSSException e) {
                continue;
            }

            final ECEFPosition estimatedPosition = estimation.getEcefPosition();
            final ECEFVelocity estimatedVelocity = estimation.getEcefVelocity();

            final double diffX = Math.abs(ecefUserPosition.getX()
                    - estimatedPosition.getX());
            final double diffY = Math.abs(ecefUserPosition.getY()
                    - estimatedPosition.getY());
            final double diffZ = Math.abs(ecefUserPosition.getZ()
                    - estimatedPosition.getZ());
            final double posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition, POSITION_ERROR));

            final double diffVx = Math.abs(ecefUserVelocity.getVx()
                    - estimatedVelocity.getVx());
            final double diffVy = Math.abs(ecefUserVelocity.getVy()
                    - estimatedVelocity.getVy());
            final double diffVz = Math.abs(ecefUserVelocity.getVz()
                    - estimatedVelocity.getVz());
            final double velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity, VELOCITY_ERROR));

            assertEquals(mEstimateStart, 1);
            assertEquals(mEstimateEnd, 1);
            assertFalse(estimator.isRunning());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onEstimateStart(final GNSSLeastSquaresPositionAndVelocityEstimator estimator) {
        checkLocked(estimator);
        mEstimateStart++;
    }

    @Override
    public void onEstimateEnd(final GNSSLeastSquaresPositionAndVelocityEstimator estimator) {
        checkLocked(estimator);
        mEstimateEnd++;
    }

    private void reset() {
        mEstimateStart = 0;
        mEstimateEnd = 0;
    }

    private void checkLocked(final GNSSLeastSquaresPositionAndVelocityEstimator estimator) {
        assertTrue(estimator.isRunning());
        try {
            estimator.setMeasurements(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setPriorPositionAndVelocity(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setPriorPositionAndVelocityFromEstimation(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setConvergenceThreshold(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.estimate(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
    }

    private static GNSSConfig generateConfig() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final int numberOfSatellites = randomizer.nextInt(MIN_SATELLITES,
                MAX_SATELLITES);
        final double orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double constellationLongitudeOffsetDegrees = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double constellationTimingOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double maskAngleDegrees = randomizer.nextDouble(
                MIN_MASK_ANGLE_DEGREES, MAX_MASK_ANGLE_DEGREES);
        final double sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double zenithIonosphereErrorSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);
        final double initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE,
                MAX_VALUE);

        return new GNSSConfig(epochInterval, initialEstimatedEcefPositionX,
                initialEstimatedEcefPositionY, initialEstimatedEcefPositionZ,
                numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees,
                constellationTimingOffset, maskAngleDegrees, sisErrorSD,
                zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset,
                initialReceiverClockDrift);
    }
}
