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
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class GNSSLeastSquaresPositionAndVelocityEstimatorTest implements GNSSLeastSquaresPositionAndVelocityEstimatorListener {

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
    private static final double MAX_PRIOR_VELOCITY_ERROR = 1.0;

    private static final int TIMES = 100;

    private int estimateStart;
    private int estimateEnd;

    @BeforeEach
    void setUp() {
        reset();
    }

    @Test
    void testConstructor() {
        // test empty constructor
        var estimator = new GNSSLeastSquaresPositionAndVelocityEstimator();

        // check default values
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getPriorPositionAndVelocity());
        assertNull(estimator.getListener());
        assertEquals(GNSSLeastSquaresPositionAndVelocityEstimator.CONVERGENCE_THRESHOLD,
                estimator.getConvergenceThreshold(), 0.0);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());


        // test constructor with measurements
        final var measurement1 = new GNSSMeasurement();
        final var measurement2 = new GNSSMeasurement();
        final var measurement3 = new GNSSMeasurement();
        final var measurement4 = new GNSSMeasurement();
        final var measurements = new ArrayList<GNSSMeasurement>();
        measurements.add(measurement1);
        measurements.add(measurement2);
        measurements.add(measurement3);
        measurements.add(measurement4);

        estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(measurements);

        // check default values
        assertSame(measurements, estimator.getMeasurements());
        assertNull(estimator.getPriorPositionAndVelocity());
        assertNull(estimator.getListener());
        assertEquals(GNSSLeastSquaresPositionAndVelocityEstimator.CONVERGENCE_THRESHOLD,
                estimator.getConvergenceThreshold(), 0.0);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());

        // Force IllegalArgumentException
        final var emptyList = Collections.<GNSSMeasurement>emptyList();
        assertThrows(IllegalArgumentException.class, () -> new GNSSLeastSquaresPositionAndVelocityEstimator(
                emptyList));

        // test constructor with measurements and prior position and velocity
        final var priorPositionAndVelocity = new ECEFPositionAndVelocity();

        estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(measurements, priorPositionAndVelocity);

        // check default values
        assertSame(measurements, estimator.getMeasurements());
        assertSame(priorPositionAndVelocity, estimator.getPriorPositionAndVelocity());
        assertNull(estimator.getListener());
        assertEquals(GNSSLeastSquaresPositionAndVelocityEstimator.CONVERGENCE_THRESHOLD,
                estimator.getConvergenceThreshold(), 0.0);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new GNSSLeastSquaresPositionAndVelocityEstimator(emptyList,
                priorPositionAndVelocity));

        // test constructor with measurements and prior result
        final var priorEstimation = new GNSSEstimation();
        estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(measurements, priorEstimation);

        // check default values
        assertSame(measurements, estimator.getMeasurements());
        assertNotNull(estimator.getPriorPositionAndVelocity());
        assertNull(estimator.getListener());
        assertEquals(GNSSLeastSquaresPositionAndVelocityEstimator.CONVERGENCE_THRESHOLD,
                estimator.getConvergenceThreshold(), 0.0);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new GNSSLeastSquaresPositionAndVelocityEstimator(
                emptyList, priorEstimation));

        // test constructor with listener
        estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(this);

        // check default values
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getPriorPositionAndVelocity());
        assertSame(this, estimator.getListener());
        assertEquals(GNSSLeastSquaresPositionAndVelocityEstimator.CONVERGENCE_THRESHOLD,
                estimator.getConvergenceThreshold(), 0.0);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());

        // test constructor with measurements and listener
        estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(measurements, this);

        // check default values
        assertSame(measurements, estimator.getMeasurements());
        assertNull(estimator.getPriorPositionAndVelocity());
        assertSame(this, estimator.getListener());
        assertEquals(GNSSLeastSquaresPositionAndVelocityEstimator.CONVERGENCE_THRESHOLD,
                estimator.getConvergenceThreshold(), 0.0);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new GNSSLeastSquaresPositionAndVelocityEstimator(
                emptyList, this));

        // test constructor with measurements, prior position and velocity and listener
        estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(measurements, priorPositionAndVelocity,
                this);

        // check default values
        assertSame(measurements, estimator.getMeasurements());
        assertSame(priorPositionAndVelocity, estimator.getPriorPositionAndVelocity());
        assertSame(this, estimator.getListener());
        assertEquals(GNSSLeastSquaresPositionAndVelocityEstimator.CONVERGENCE_THRESHOLD,
                estimator.getConvergenceThreshold(), 0.0);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new GNSSLeastSquaresPositionAndVelocityEstimator(
                emptyList, priorPositionAndVelocity, this));

        // test constructor with measurements and prior result
        estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(measurements, priorEstimation, this);

        // check default values
        assertSame(measurements, estimator.getMeasurements());
        assertNotNull(estimator.getPriorPositionAndVelocity());
        assertSame(this, estimator.getListener());
        assertEquals(GNSSLeastSquaresPositionAndVelocityEstimator.CONVERGENCE_THRESHOLD,
                estimator.getConvergenceThreshold(), 0.0);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> new GNSSLeastSquaresPositionAndVelocityEstimator(
                emptyList, priorEstimation, this));
    }

    @Test
    void testGetSetMeasurements() throws LockedException {
        final var estimator = new GNSSLeastSquaresPositionAndVelocityEstimator();

        // check default value
        assertNull(estimator.getMeasurements());

        // set new value
        final var measurement1 = new GNSSMeasurement();
        final var measurement2 = new GNSSMeasurement();
        final var measurement3 = new GNSSMeasurement();
        final var measurement4 = new GNSSMeasurement();
        final var measurements = new ArrayList<GNSSMeasurement>();
        measurements.add(measurement1);
        measurements.add(measurement2);
        measurements.add(measurement3);
        measurements.add(measurement4);

        estimator.setMeasurements(measurements);

        // check
        assertSame(measurements, estimator.getMeasurements());

        // Force IllegalArgumentException
        final var emptyList = Collections.<GNSSMeasurement>emptyList();
        assertThrows(IllegalArgumentException.class, () -> estimator.setMeasurements(emptyList));
    }

    @Test
    void testGetSetPriorPositionAndVelocity() throws LockedException {
        final var estimator = new GNSSLeastSquaresPositionAndVelocityEstimator();

        // check default value
        assertNull(estimator.getPriorPositionAndVelocity());

        // set new value
        final var priorPositionAndVelocity = new ECEFPositionAndVelocity();

        estimator.setPriorPositionAndVelocity(priorPositionAndVelocity);

        // check
        assertSame(priorPositionAndVelocity, estimator.getPriorPositionAndVelocity());
    }

    @Test
    void testSetPriorPositionAndVelocityFromEstimation() throws LockedException {
        final var estimator = new GNSSLeastSquaresPositionAndVelocityEstimator();

        // set new value
        final var priorEstimation = new GNSSEstimation();
        final var priorPositionAndVelocity = new ECEFPositionAndVelocity();
        priorPositionAndVelocity.setPositionCoordinates(1.0, 2.0, 3.0);
        priorEstimation.setPositionAndVelocity(priorPositionAndVelocity);

        estimator.setPriorPositionAndVelocityFromEstimation(priorEstimation);

        // check
        assertEquals(priorPositionAndVelocity, estimator.getPriorPositionAndVelocity());


        estimator.setPriorPositionAndVelocityFromEstimation(null);

        // check
        assertNull(estimator.getPriorPositionAndVelocity());
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new GNSSLeastSquaresPositionAndVelocityEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testGetSetConvergenceThreshold() throws LockedException {
        final var estimator = new GNSSLeastSquaresPositionAndVelocityEstimator();

        // check default value
        assertEquals(GNSSLeastSquaresPositionAndVelocityEstimator.CONVERGENCE_THRESHOLD,
                estimator.getConvergenceThreshold(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var threshold = randomizer.nextDouble(MIN_CONVERGENCE, MAX_CONVERGENCE);
        estimator.setConvergenceThreshold(threshold);

        // check
        assertEquals(threshold, estimator.getConvergenceThreshold(), 0.0);
    }

    @Test
    void testIsValidMeasurements() {
        assertFalse(GNSSLeastSquaresPositionAndVelocityEstimator.isValidMeasurements(Collections.emptyList()));

        final var measurement1 = new GNSSMeasurement();
        final var measurement2 = new GNSSMeasurement();
        final var measurement3 = new GNSSMeasurement();
        final var measurement4 = new GNSSMeasurement();
        final var measurements = new ArrayList<GNSSMeasurement>();
        measurements.add(measurement1);
        measurements.add(measurement2);
        measurements.add(measurement3);
        measurements.add(measurement4);

        assertTrue(GNSSLeastSquaresPositionAndVelocityEstimator.isValidMeasurements(measurements));
    }

    @Test
    void testEstimateWithoutPriorPositionAndVelocity() throws LockedException, NotReadyException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var numSatellites = randomizer.nextInt(MIN_NUM_SAT, MAX_NUM_SAT);
            final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

            final var userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES,
                    MAX_LONGITUDE_DEGREES));
            final var userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
            final var nedUserPosition = new NEDPosition(userLatitude, userLongitude, userHeight);

            final var userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final var userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final var userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final var nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

            final var ecefUserPosition = new ECEFPosition();
            final var ecefUserVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                    ecefUserVelocity);

            final var ecefUserPositionAndVelocity = new ECEFPositionAndVelocity(ecefUserPosition, ecefUserVelocity);

            final var config = generateConfig();
            final var maskAngle = Math.toRadians(config.getMaskAngleDegrees());
            final var delta = maskAngle / 3.0;

            final var biases = new ArrayList<Double>();
            final var satellitePositionsAndVelocities = new ArrayList<ECEFPositionAndVelocity>();
            final var random = new Random();
            for (var n = 0; n < numSatellites; n++) {
                final var satLatitude = randomizer.nextDouble(userLatitude - delta, userLatitude + delta);
                final var satLongitude = randomizer.nextDouble(userLongitude - delta, userLongitude + delta);
                final var satHeight = randomizer.nextDouble(MIN_SAT_HEIGHT, MAX_SAT_HEIGHT);
                final var nedSatPosition = new NEDPosition(satLatitude, satLongitude, satHeight);

                final var satVn = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final var satVe = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final var satVd = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final var nedSatVelocity = new NEDVelocity(satVn, satVe, satVd);

                final var ecefSatPosition = new ECEFPosition();
                final var ecefSatVelocity = new ECEFVelocity();
                NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedSatPosition, nedSatVelocity, ecefSatPosition,
                        ecefSatVelocity);

                final var ecefSatPositionAndVelocity = new ECEFPositionAndVelocity(ecefSatPosition, ecefSatVelocity);

                final var bias = GNSSBiasesGenerator.generateBias(ecefSatPosition, ecefUserPosition, config, random);

                biases.add(bias);
                satellitePositionsAndVelocities.add(ecefSatPositionAndVelocity);
            }

            final var measurements = GNSSMeasurementsGenerator.generate(timeSeconds, satellitePositionsAndVelocities,
                    ecefUserPositionAndVelocity, biases, config, random);

            if (measurements.size() < GNSSLeastSquaresPositionAndVelocityEstimator.MIN_MEASUREMENTS) {
                continue;
            }

            final GNSSLeastSquaresPositionAndVelocityEstimator estimator =
                    new GNSSLeastSquaresPositionAndVelocityEstimator(measurements, this);

            // check initial values
            reset();
            assertFalse(estimator.isRunning());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            GNSSEstimation estimation;
            try {
                estimation = estimator.estimate();
            } catch (final GNSSException e) {
                continue;
            }

            final var estimatedPosition = estimation.getEcefPosition();
            final var estimatedVelocity = estimation.getEcefVelocity();

            final var diffX = Math.abs(ecefUserPosition.getX() - estimatedPosition.getX());
            final var diffY = Math.abs(ecefUserPosition.getY() - estimatedPosition.getY());
            final var diffZ = Math.abs(ecefUserPosition.getZ() - estimatedPosition.getZ());
            final var posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition, POSITION_ERROR));

            final var diffVx = Math.abs(ecefUserVelocity.getVx() - estimatedVelocity.getVx());
            final var diffVy = Math.abs(ecefUserVelocity.getVy() - estimatedVelocity.getVy());
            final var diffVz = Math.abs(ecefUserVelocity.getVz() - estimatedVelocity.getVz());
            final var velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity, VELOCITY_ERROR));

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertFalse(estimator.isRunning());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testEstimateWithPriorPositionAndVelocity() throws LockedException, NotReadyException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var random = new Random();
            final var randomizer = new UniformRandomizer(random);

            final var numSatellites = randomizer.nextInt(MIN_NUM_SAT, MAX_NUM_SAT);
            final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

            final var userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES,
                    MAX_LONGITUDE_DEGREES));
            final var userHeight = randomizer.nextDouble(MIN_USER_HEIGHT, MAX_USER_HEIGHT);
            final var nedUserPosition = new NEDPosition(userLatitude, userLongitude, userHeight);

            final var userVn = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final var userVe = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final var userVd = randomizer.nextDouble(MIN_USER_VELOCITY_VALUE, MAX_USER_VELOCITY_VALUE);
            final var nedUserVelocity = new NEDVelocity(userVn, userVe, userVd);

            final var ecefUserPosition = new ECEFPosition();
            final var ecefUserVelocity = new ECEFVelocity();
            NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedUserPosition, nedUserVelocity, ecefUserPosition,
                    ecefUserVelocity);

            final var ecefUserPositionAndVelocity = new ECEFPositionAndVelocity(ecefUserPosition, ecefUserVelocity);

            final var config = generateConfig();
            final var maskAngle = Math.toRadians(config.getMaskAngleDegrees());
            final var delta = maskAngle / 3.0;

            final var biases = new ArrayList<Double>();
            final var satellitePositionsAndVelocities = new ArrayList<ECEFPositionAndVelocity>();
            for (var n = 0; n < numSatellites; n++) {
                final var satLatitude = randomizer.nextDouble(userLatitude - delta, userLatitude + delta);
                final var satLongitude = randomizer.nextDouble(userLongitude - delta, userLongitude + delta);
                final var satHeight = randomizer.nextDouble(MIN_SAT_HEIGHT, MAX_SAT_HEIGHT);
                final var nedSatPosition = new NEDPosition(satLatitude, satLongitude, satHeight);

                final var satVn = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final var satVe = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final var satVd = randomizer.nextDouble(MIN_SAT_VELOCITY_VALUE, MAX_SAT_VELOCITY_VALUE);
                final var nedSatVelocity = new NEDVelocity(satVn, satVe, satVd);

                final var ecefSatPosition = new ECEFPosition();
                final var ecefSatVelocity = new ECEFVelocity();
                NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedSatPosition, nedSatVelocity, ecefSatPosition,
                        ecefSatVelocity);

                final var ecefSatPositionAndVelocity = new ECEFPositionAndVelocity(ecefSatPosition, ecefSatVelocity);

                final var bias = GNSSBiasesGenerator.generateBias(ecefSatPosition, ecefUserPosition, config, random);

                biases.add(bias);
                satellitePositionsAndVelocities.add(ecefSatPositionAndVelocity);
            }

            final var measurements = GNSSMeasurementsGenerator.generate(timeSeconds, satellitePositionsAndVelocities,
                    ecefUserPositionAndVelocity, biases, config, random);

            if (measurements.size() < GNSSLeastSquaresPositionAndVelocityEstimator.MIN_MEASUREMENTS) {
                continue;
            }

            final var priorX = ecefUserPosition.getX() + randomizer.nextDouble(MIN_PRIOR_POSITION_ERROR,
                    MAX_PRIOR_POSITION_ERROR);
            final var priorY = ecefUserPosition.getY() + randomizer.nextDouble(MIN_PRIOR_POSITION_ERROR,
                    MAX_PRIOR_POSITION_ERROR);
            final var priorZ = ecefUserPosition.getZ() + randomizer.nextDouble(MIN_PRIOR_POSITION_ERROR,
                    MAX_PRIOR_POSITION_ERROR);
            final var priorVx = ecefUserVelocity.getVx() + randomizer.nextDouble(MIN_PRIOR_VELOCITY_ERROR,
                    MAX_PRIOR_VELOCITY_ERROR);
            final var priorVy = ecefUserVelocity.getVy() + randomizer.nextDouble(MIN_PRIOR_VELOCITY_ERROR,
                    MAX_PRIOR_VELOCITY_ERROR);
            final var priorVz = ecefUserVelocity.getVz() + randomizer.nextDouble(MIN_PRIOR_VELOCITY_ERROR,
                    MAX_PRIOR_VELOCITY_ERROR);

            final var priorPositionAndVelocity = new ECEFPositionAndVelocity(priorX, priorY, priorZ, priorVx, priorVy,
                    priorVz);

            final var estimator = new GNSSLeastSquaresPositionAndVelocityEstimator(measurements,
                    priorPositionAndVelocity, this);

            // check initial values
            reset();
            assertFalse(estimator.isRunning());
            assertEquals(0, estimateStart);
            assertEquals(0, estimateEnd);

            GNSSEstimation estimation;
            try {
                estimation = estimator.estimate();
            } catch (final GNSSException e) {
                continue;
            }

            final var estimatedPosition = estimation.getEcefPosition();
            final var estimatedVelocity = estimation.getEcefVelocity();

            final var diffX = Math.abs(ecefUserPosition.getX() - estimatedPosition.getX());
            final var diffY = Math.abs(ecefUserPosition.getY() - estimatedPosition.getY());
            final var diffZ = Math.abs(ecefUserPosition.getZ() - estimatedPosition.getZ());
            final var posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition, POSITION_ERROR));

            final var diffVx = Math.abs(ecefUserVelocity.getVx() - estimatedVelocity.getVx());
            final var diffVy = Math.abs(ecefUserVelocity.getVy() - estimatedVelocity.getVy());
            final var diffVz = Math.abs(ecefUserVelocity.getVz() - estimatedVelocity.getVz());
            final var velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity, VELOCITY_ERROR));

            assertEquals(1, estimateStart);
            assertEquals(1, estimateEnd);
            assertFalse(estimator.isRunning());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onEstimateStart(final GNSSLeastSquaresPositionAndVelocityEstimator estimator) {
        checkLocked(estimator);
        estimateStart++;
    }

    @Override
    public void onEstimateEnd(final GNSSLeastSquaresPositionAndVelocityEstimator estimator) {
        checkLocked(estimator);
        estimateEnd++;
    }

    private void reset() {
        estimateStart = 0;
        estimateEnd = 0;
    }

    private static void checkLocked(final GNSSLeastSquaresPositionAndVelocityEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setMeasurements(null));
        assertThrows(LockedException.class, () -> estimator.setPriorPositionAndVelocity(null));
        assertThrows(LockedException.class, () -> estimator.setPriorPositionAndVelocityFromEstimation(null));
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setConvergenceThreshold(0.0));
        assertThrows(LockedException.class, () -> estimator.estimate(null));
    }

    private static GNSSConfig generateConfig() {
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var numberOfSatellites = randomizer.nextInt(MIN_SATELLITES, MAX_SATELLITES);
        final var orbitalRadiusOfSatellites = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var satellitesInclinationDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationLongitudeOffsetDegrees = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var constellationTimingOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var maskAngleDegrees = randomizer.nextDouble(MIN_MASK_ANGLE_DEGREES, MAX_MASK_ANGLE_DEGREES);
        final var sisErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithIonosphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var zenithTroposphereErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var codeTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateTrackingErrorSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockOffset = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialReceiverClockDrift = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        return new GNSSConfig(epochInterval, initialEstimatedEcefPositionX, initialEstimatedEcefPositionY,
                initialEstimatedEcefPositionZ, numberOfSatellites, orbitalRadiusOfSatellites,
                satellitesInclinationDegrees, constellationLongitudeOffsetDegrees, constellationTimingOffset,
                maskAngleDegrees, sisErrorSD, zenithIonosphereErrorSD, zenithTroposphereErrorSD, codeTrackingErrorSD,
                rangeRateTrackingErrorSD, initialReceiverClockOffset, initialReceiverClockDrift);
    }
}
