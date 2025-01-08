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

import com.irurueta.algebra.Utils;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class GNSSKalmanFilteredEstimatorTest implements GNSSKalmanFilteredEstimatorListener {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final double MIN_EPOCH_INTERVAL = 1e-5;
    private static final double MAX_EPOCH_INTERVAL = 1.0;

    private static final int MIN_NUM_SAT = 4;
    private static final int MAX_NUM_SAT = 10;

    private static final double MIN_TIME = 0.0;
    private static final double MAX_TIME = 1.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_USER_HEIGHT = -50.0;
    private static final double MAX_USER_HEIGHT = 50.0;

    private static final double MIN_USER_VELOCITY_VALUE = -2.0;
    private static final double MAX_USER_VELOCITY_VALUE = 2.0;

    private static final double MIN_SAT_HEIGHT = 150000.0;
    private static final double MAX_SAT_HEIGHT = 800000.0;

    private static final double MIN_SAT_VELOCITY_VALUE = -3500.0;
    private static final double MAX_SAT_VELOCITY_VALUE = 3500.0;

    private static final double MIN_MASK_ANGLE_DEGREES = 15.0;
    private static final double MAX_MASK_ANGLE_DEGREES = 20.0;

    private static final double POSITION_ERROR = 5e-1;
    private static final double VELOCITY_ERROR = 5e-2;

    private static final double PROPAGATION_ERROR = 1.0;

    private static final int TIMES = 100;

    private int updateStart;
    private int updateEnd;
    private int propagateStart;
    private int propagateEnd;
    private int reset;

    @Test
    void testConstructor() {

        // test empty constructor
        var estimator = new GNSSKalmanFilteredEstimator();

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        var epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        final var lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor with configuration
        final var config = generateKalmanConfig();
        estimator = new GNSSKalmanFilteredEstimator(config);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        var config2 = new GNSSKalmanConfig();
        assertTrue(estimator.getConfig(config2));
        assertEquals(config, config2);
        assertEquals(config, estimator.getConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor with epoch interval
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_EPOCH_INTERVAL, MAX_EPOCH_INTERVAL);
        estimator = new GNSSKalmanFilteredEstimator(epochInterval);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(config2));
        assertNull(estimator.getConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor with listener
        estimator = new GNSSKalmanFilteredEstimator(this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor with config and epoch interval
        estimator = new GNSSKalmanFilteredEstimator(config, epochInterval);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertTrue(estimator.getConfig(config2));
        assertEquals(config, config2);
        assertEquals(config, estimator.getConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor with config and listener
        estimator = new GNSSKalmanFilteredEstimator(config, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), epochIntervalTime);
        assertEquals(new Time(0.0, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertTrue(estimator.getConfig(config2));
        assertEquals(config, config2);
        assertEquals(config, estimator.getConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor with epoch interval and listener
        estimator = new GNSSKalmanFilteredEstimator(epochInterval, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(config2));
        assertNull(estimator.getConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor with config, epoch interval and listener
        estimator = new GNSSKalmanFilteredEstimator(config, epochInterval, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertTrue(estimator.getConfig(config2));
        assertEquals(config, config2);
        assertEquals(config, estimator.getConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor with epoch interval time
        epochIntervalTime = new Time(epochInterval, TimeUnit.SECOND);
        estimator = new GNSSKalmanFilteredEstimator(epochIntervalTime);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(config2));
        assertNull(estimator.getConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor with config en epoch interval time
        estimator = new GNSSKalmanFilteredEstimator(config, epochIntervalTime);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertTrue(estimator.getConfig(config2));
        assertEquals(config, config2);
        assertEquals(config, estimator.getConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test cons constructor with epoch interval time and listener
        estimator = new GNSSKalmanFilteredEstimator(epochIntervalTime, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertFalse(estimator.getConfig(config2));
        assertNull(estimator.getConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // test constructor with config, epoch interval time and listener
        estimator = new GNSSKalmanFilteredEstimator(config, epochIntervalTime, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(new Time(epochInterval, TimeUnit.SECOND), estimator.getEpochIntervalAsTime());
        assertTrue(estimator.getConfig(config2));
        assertEquals(config, config2);
        assertEquals(config, estimator.getConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new GNSSKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testGetSetEpochInterval() throws LockedException {
        final var estimator = new GNSSKalmanFilteredEstimator();

        // check default value
        assertEquals(0.0, estimator.getEpochInterval(), 0.0);

        // set new value
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_EPOCH_INTERVAL, MAX_EPOCH_INTERVAL);
        estimator.setEpochInterval(epochInterval);

        // check
        assertEquals(epochInterval, estimator.getEpochInterval(), 0.0);
    }

    @Test
    void testGetSetEpochIntervalAsTime() throws LockedException {
        final var estimator = new GNSSKalmanFilteredEstimator();

        // check default value
        final var epochInterval1 = estimator.getEpochIntervalAsTime();

        assertEquals(0.0, epochInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, epochInterval1.getUnit());

        // set new value
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_EPOCH_INTERVAL, MAX_EPOCH_INTERVAL);
        final var epochInterval2 = new Time(epochInterval, TimeUnit.SECOND);
        estimator.setEpochInterval(epochInterval2);

        // check
        final var epochInterval3 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochInterval3);
        final var epochInterval4 = estimator.getEpochIntervalAsTime();

        assertEquals(epochInterval2, epochInterval3);
        assertEquals(epochInterval2, epochInterval4);
    }

    @Test
    void testGetSetConfig() throws LockedException {
        final var estimator = new GNSSKalmanFilteredEstimator();

        // check default value
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());

        // set new value
        final var config1 = generateKalmanConfig();
        estimator.setConfig(config1);

        // check
        final var config2 = new GNSSKalmanConfig();
        assertTrue(estimator.getConfig(config2));
        final var config3 = estimator.getConfig();

        assertEquals(config1, config2);
        assertEquals(config1, config3);
    }

    @Test
    void testIsUpdateMeasurementsReady() {
        //noinspection ConstantConditions
        assertFalse(GNSSKalmanFilteredEstimator.isUpdateMeasurementsReady(null));

        final var measurements = new ArrayList<GNSSMeasurement>();
        assertFalse(GNSSKalmanFilteredEstimator.isUpdateMeasurementsReady(measurements));

        for (var i = 0; i < GNSSLeastSquaresPositionAndVelocityEstimator.MIN_MEASUREMENTS; i++) {
            measurements.add(new GNSSMeasurement());
        }
        assertTrue(GNSSKalmanFilteredEstimator.isUpdateMeasurementsReady(measurements));
    }

    @Test
    void testUpdateMeasurements() throws LockedException, NotReadyException, GNSSException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

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
            final var numSatellites = config.getNumberOfSatellites();
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

            final var kalmanConfig = generateKalmanConfig();
            final var estimator = new GNSSKalmanFilteredEstimator(kalmanConfig, this);

            reset();
            assertEquals(0, updateStart);
            assertEquals(0, updateEnd);
            assertEquals(0, propagateStart);
            assertEquals(0, propagateEnd);

            assertNull(estimator.getEstimation());
            assertFalse(estimator.getEstimation(null));
            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));

            // update measurement
            try {
                assertTrue(estimator.updateMeasurements(measurements, timeSeconds));
            } catch (final GNSSException e) {
                continue;
            }

            assertFalse(estimator.isRunning());
            assertEquals(measurements, estimator.getMeasurements());
            assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            assertEquals(new Time(timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
            final var timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(timestamp, new Time(timeSeconds, TimeUnit.SECOND));

            final var estimation1 = estimator.getEstimation();
            assertNotNull(estimation1);

            final var estimation2 = new GNSSEstimation();
            assertTrue(estimator.getEstimation(estimation2));

            assertEquals(estimation1, estimation2);

            final var state1 = estimator.getState();
            assertNotNull(state1);

            final var state2 = new GNSSKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            assertEquals(estimation1, state1.getEstimation());

            final var estimatedPosition = estimation1.getEcefPosition();
            final var estimatedVelocity = estimation1.getEcefVelocity();

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

            // update again with same timestamp makes no action
            assertFalse(estimator.updateMeasurements(measurements, new Time(timeSeconds, TimeUnit.SECOND)));

            assertFalse(estimator.isRunning());
            assertEquals(estimator.getLastStateTimestamp(), timeSeconds, 0.0);
            final var estimation3 = estimator.getEstimation();
            final var state3 = estimator.getState();

            assertEquals(estimation1, estimation3);
            assertEquals(state1, state3);

            assertEquals(1, updateStart);
            assertEquals(1, updateEnd);
            assertEquals(1, propagateStart);
            assertEquals(1, propagateEnd);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testUpdateWhenNotReadyThrowsNotReadyException() {

        final var kalmanConfig = generateKalmanConfig();
        final var estimator = new GNSSKalmanFilteredEstimator(kalmanConfig);

        assertThrows(NotReadyException.class,
                () -> estimator.updateMeasurements(Collections.emptyList(), 0.0));
    }

    @Test
    void testPropagate() throws LockedException, NotReadyException, GNSSException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

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
            final var numSatellites = config.getNumberOfSatellites();
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

            final var kalmanConfig = generateKalmanConfig();
            final var estimator = new GNSSKalmanFilteredEstimator(kalmanConfig, this);

            reset();
            assertEquals(0, updateStart);
            assertEquals(0, updateEnd);
            assertEquals(0, propagateStart);
            assertEquals(0, propagateEnd);

            assertNull(estimator.getEstimation());
            assertFalse(estimator.getEstimation(null));
            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));

            // update measurement
            try {
                assertTrue(estimator.updateMeasurements(measurements, timeSeconds));
            } catch (final GNSSException e) {
                continue;
            }

            assertEquals(measurements, estimator.getMeasurements());
            assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            final var estimation1 = estimator.getEstimation();
            assertNotNull(estimation1);

            final var estimation2 = new GNSSEstimation();
            assertTrue(estimator.getEstimation(estimation2));

            assertEquals(estimation1, estimation2);

            final var state1 = estimator.getState();
            assertNotNull(state1);

            final var state2 = new GNSSKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            assertEquals(estimation1, state1.getEstimation());

            final var estimatedPosition1 = estimation1.getEcefPosition();
            final var estimatedVelocity1 = estimation1.getEcefVelocity();

            final var diffX1 = Math.abs(ecefUserPosition.getX() - estimatedPosition1.getX());
            final var diffY1 = Math.abs(ecefUserPosition.getY() - estimatedPosition1.getY());
            final var diffZ1 = Math.abs(ecefUserPosition.getZ() - estimatedPosition1.getZ());
            final var posError1 = Math.max(diffX1, Math.max(diffY1, diffZ1));
            if (posError1 > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition1, POSITION_ERROR));

            final var diffVx1 = Math.abs(ecefUserVelocity.getVx() - estimatedVelocity1.getVx());
            final var diffVy1 = Math.abs(ecefUserVelocity.getVy() - estimatedVelocity1.getVy());
            final var diffVz1 = Math.abs(ecefUserVelocity.getVz() - estimatedVelocity1.getVz());
            final var velError = Math.max(diffVx1, Math.max(diffVy1, diffVz1));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity1, VELOCITY_ERROR));

            assertEquals(1, updateStart);
            assertEquals(1, updateEnd);
            assertEquals(1, propagateStart);
            assertEquals(1, propagateEnd);

            // propagate
            assertTrue(estimator.propagate(2.0 * timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(2.0 * timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            final var estimation3 = estimator.getEstimation();
            final var state3 = estimator.getState();

            assertEquals(estimation3, state3.getEstimation());

            final var estimatedPosition3 = estimation3.getEcefPosition();
            final var estimatedVelocity3 = estimation3.getEcefVelocity();

            final var diffX3 = Math.abs(ecefUserPosition.getX() - estimatedPosition3.getX());
            final var diffY3 = Math.abs(ecefUserPosition.getY() - estimatedPosition3.getY());
            final var diffZ3 = Math.abs(ecefUserPosition.getZ() - estimatedPosition3.getZ());
            final var posError3 = Math.max(diffX3, Math.max(diffY3, diffZ3));
            if (posError3 > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition3, PROPAGATION_ERROR));

            final var diffVx3 = Math.abs(ecefUserVelocity.getVx() - estimatedVelocity3.getVx());
            final var diffVy3 = Math.abs(ecefUserVelocity.getVy() - estimatedVelocity3.getVy());
            final var diffVz3 = Math.abs(ecefUserVelocity.getVz() - estimatedVelocity3.getVz());
            final var velError3 = Math.max(diffVx3, Math.max(diffVy3, diffVz3));
            if (velError3 > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity3, PROPAGATION_ERROR));

            final var covariance1 = state1.getCovariance();
            final var covariance3 = state3.getCovariance();

            final var norm1 = Utils.normF(covariance1);
            final var norm3 = Utils.normF(covariance3);
            if (norm3 < norm1) {
                continue;
            }
            assertTrue(norm3 >= norm1);

            assertFalse(estimator.propagate(new Time(2.0 * timeSeconds, TimeUnit.SECOND)));

            assertFalse(estimator.isRunning());
            assertEquals(2.0 * timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            final var estimation4 = estimator.getEstimation();
            final var state4 = estimator.getState();

            assertEquals(estimation3, estimation4);
            assertEquals(state3, state4);

            assertEquals(1, updateStart);
            assertEquals(1, updateEnd);
            assertEquals(2, propagateStart);
            assertEquals(2, propagateEnd);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testPropagateWhenNotReadyThrowsNotReadyException() {

        final var kalmanConfig = generateKalmanConfig();
        final var estimator = new GNSSKalmanFilteredEstimator(kalmanConfig);

        assertThrows(NotReadyException.class, () -> estimator.propagate(0.0));
    }

    @Test
    void testReset() throws LockedException, NotReadyException, GNSSException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var randomizer = new UniformRandomizer();

            final var timeSeconds = randomizer.nextDouble(MIN_TIME, MAX_TIME);

            final var userLatitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final var userLongitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
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
            final var numSatellites = config.getNumberOfSatellites();
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

            final var kalmanConfig = generateKalmanConfig();
            final var estimator = new GNSSKalmanFilteredEstimator(kalmanConfig, this);

            reset();
            assertEquals(0, updateStart);
            assertEquals(0, updateEnd);
            assertEquals(0, propagateStart);
            assertEquals(0, propagateEnd);

            assertNull(estimator.getEstimation());
            assertFalse(estimator.getEstimation(null));
            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));

            // update measurement
            try {
                assertTrue(estimator.updateMeasurements(measurements, timeSeconds));
            } catch (final GNSSException e) {
                continue;
            }

            assertFalse(estimator.isRunning());
            assertEquals(measurements, estimator.getMeasurements());
            assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            assertEquals(new Time(timeSeconds, TimeUnit.SECOND), estimator.getLastStateTimestampAsTime());
            final var timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(timestamp, new Time(timeSeconds, TimeUnit.SECOND));

            final var estimation1 = estimator.getEstimation();
            assertNotNull(estimation1);

            final var state1 = estimator.getState();
            assertNotNull(state1);

            assertEquals(estimation1, state1.getEstimation());

            final var estimatedPosition = estimation1.getEcefPosition();
            final var estimatedVelocity = estimation1.getEcefVelocity();

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

            // reset
            assertEquals(0, reset);

            estimator.reset();

            assertEquals(1, reset);
            assertNull(estimator.getMeasurements());
            assertNull(estimator.getEstimation());
            assertNull(estimator.getState());
            assertNull(estimator.getLastStateTimestamp());
            assertFalse(estimator.isRunning());

            // update again with same timestamp now it does make an action
            assertTrue(estimator.updateMeasurements(measurements, new Time(timeSeconds, TimeUnit.SECOND)));

            assertEquals(measurements, estimator.getMeasurements());
            assertEquals(timeSeconds, estimator.getLastStateTimestamp(), 0.0);
            final var estimation2 = estimator.getEstimation();
            final var state2 = estimator.getState();

            assertEquals(estimation1, estimation2);
            assertEquals(state1, state2);

            assertEquals(2, updateStart);
            assertEquals(2, updateEnd);
            assertEquals(2, propagateStart);
            assertEquals(2, propagateEnd);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onUpdateStart(final GNSSKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        updateStart++;
    }

    @Override
    public void onUpdateEnd(final GNSSKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        updateEnd++;
    }

    @Override
    public void onPropagateStart(final GNSSKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        propagateStart++;
    }

    @Override
    public void onPropagateEnd(final GNSSKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        propagateEnd++;
    }

    @Override
    public void onReset(final GNSSKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        reset++;
    }

    private void reset() {
        updateStart = 0;
        updateEnd = 0;
        propagateStart = 0;
        propagateEnd = 0;
        reset = 0;
    }

    private static void checkLocked(final GNSSKalmanFilteredEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setEpochInterval(0.0));
        assertThrows(LockedException.class, () -> estimator.setEpochInterval(new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.setConfig(null));
        assertThrows(LockedException.class, () -> estimator.updateMeasurements(null, 0.0));
        assertThrows(LockedException.class, () -> estimator.updateMeasurements(null,
                new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.propagate(0.0));
        assertThrows(LockedException.class, () -> estimator.propagate(new Time(0.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, estimator::reset);
    }

    private static GNSSKalmanConfig generateKalmanConfig() {
        final var randomizer = new UniformRandomizer();
        final var initialPositionUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialVelocityUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockOffsetUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialClockDriftUncertainty = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var accelerationPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockFrequencyPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var clockPhasePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var pseudoRangeSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var rangeRateSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        return new GNSSKalmanConfig(initialPositionUncertainty, initialVelocityUncertainty,
                initialClockOffsetUncertainty, initialClockDriftUncertainty, accelerationPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);
    }

    private static GNSSConfig generateConfig() {
        final var randomizer = new UniformRandomizer();
        final var epochInterval = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionX = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionY = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var initialEstimatedEcefPositionZ = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final var numberOfSatellites = randomizer.nextInt(MIN_NUM_SAT, MAX_NUM_SAT);
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
