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

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class GNSSKalmanFilteredEstimatorTest implements GNSSKalmanFilteredEstimatorListener {

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

    private int mUpdateStart;
    private int mUpdateEnd;
    private int mPropagateStart;
    private int mPropagateEnd;
    private int mReset;

    @Test
    public void testConstructor() {

        // test empty constructor
        GNSSKalmanFilteredEstimator estimator = new GNSSKalmanFilteredEstimator();

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);
        Time epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(0.0, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(0.0, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getMeasurements());
        assertNull(estimator.getEstimation());
        assertFalse(estimator.getEstimation(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        Time lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());


        // test constructor with configuration
        final GNSSKalmanConfig config = generateKalmanConfig();
        estimator = new GNSSKalmanFilteredEstimator(config);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(0.0, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(0.0, TimeUnit.SECOND));
        GNSSKalmanConfig config2 = new GNSSKalmanConfig();
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
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(
                MIN_EPOCH_INTERVAL, MAX_EPOCH_INTERVAL);
        estimator = new GNSSKalmanFilteredEstimator(epochInterval);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
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
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(0.0, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(0.0, TimeUnit.SECOND));
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
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
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
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(0.0, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(0.0, TimeUnit.SECOND));
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
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
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
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
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
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
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
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
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
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
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
        estimator = new GNSSKalmanFilteredEstimator(config, epochIntervalTime,
                this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
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
    public void testGetSetListener() throws LockedException {
        final GNSSKalmanFilteredEstimator estimator = new GNSSKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testGetSetEpochInterval() throws LockedException {
        final GNSSKalmanFilteredEstimator estimator = new GNSSKalmanFilteredEstimator();

        // check default value
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(
                MIN_EPOCH_INTERVAL, MAX_EPOCH_INTERVAL);
        estimator.setEpochInterval(epochInterval);

        // check
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
    }

    @Test
    public void testGetSetEpochIntervalAsTime() throws LockedException {
        final GNSSKalmanFilteredEstimator estimator = new GNSSKalmanFilteredEstimator();

        // check default value
        final Time epochInterval1 = estimator.getEpochIntervalAsTime();

        assertEquals(epochInterval1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(epochInterval1.getUnit(), TimeUnit.SECOND);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(
                MIN_EPOCH_INTERVAL, MAX_EPOCH_INTERVAL);
        final Time epochInterval2 = new Time(epochInterval, TimeUnit.SECOND);
        estimator.setEpochInterval(epochInterval2);

        // check
        final Time epochInterval3 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochInterval3);
        final Time epochInterval4 = estimator.getEpochIntervalAsTime();

        assertEquals(epochInterval2, epochInterval3);
        assertEquals(epochInterval2, epochInterval4);
    }

    @Test
    public void testGetSetConfig() throws LockedException {
        final GNSSKalmanFilteredEstimator estimator = new GNSSKalmanFilteredEstimator();

        // check default value
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());

        // set new value
        final GNSSKalmanConfig config1 = generateKalmanConfig();
        estimator.setConfig(config1);

        // check
        final GNSSKalmanConfig config2 = new GNSSKalmanConfig();
        assertTrue(estimator.getConfig(config2));
        final GNSSKalmanConfig config3 = estimator.getConfig();

        assertEquals(config1, config2);
        assertEquals(config1, config3);
    }

    @Test
    public void testIsUpdateMeasurementsReady() {
        //noinspection ConstantConditions
        assertFalse(GNSSKalmanFilteredEstimator.isUpdateMeasurementsReady(null));

        final List<GNSSMeasurement> measurements = new ArrayList<>();
        assertFalse(GNSSKalmanFilteredEstimator.isUpdateMeasurementsReady(measurements));

        for (int i = 0; i < GNSSLeastSquaresPositionAndVelocityEstimator.MIN_MEASUREMENTS; i++) {
            measurements.add(new GNSSMeasurement());
        }
        assertTrue(GNSSKalmanFilteredEstimator.isUpdateMeasurementsReady(measurements));
    }

    @Test
    public void testUpdateMeasurements() throws LockedException, NotReadyException,
            GNSSException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

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
            final int numSatellites = config.getNumberOfSatellites();
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

            final GNSSKalmanConfig kalmanConfig = generateKalmanConfig();
            final GNSSKalmanFilteredEstimator estimator =
                    new GNSSKalmanFilteredEstimator(kalmanConfig, this);

            reset();
            assertEquals(mUpdateStart, 0);
            assertEquals(mUpdateEnd, 0);
            assertEquals(mPropagateStart, 0);
            assertEquals(mPropagateEnd, 0);

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
            assertEquals(estimator.getMeasurements(), measurements);
            assertEquals(estimator.getLastStateTimestamp(), timeSeconds, 0.0);
            assertEquals(estimator.getLastStateTimestampAsTime(),
                    new Time(timeSeconds, TimeUnit.SECOND));
            final Time timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(timestamp, new Time(timeSeconds, TimeUnit.SECOND));

            final GNSSEstimation estimation1 = estimator.getEstimation();
            assertNotNull(estimation1);

            final GNSSEstimation estimation2 = new GNSSEstimation();
            assertTrue(estimator.getEstimation(estimation2));

            assertEquals(estimation1, estimation2);

            final GNSSKalmanState state1 = estimator.getState();
            assertNotNull(state1);

            final GNSSKalmanState state2 = new GNSSKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            assertEquals(estimation1, state1.getEstimation());

            final ECEFPosition estimatedPosition = estimation1.getEcefPosition();
            final ECEFVelocity estimatedVelocity = estimation1.getEcefVelocity();

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

            // update again with same timestamp makes no action
            assertFalse(estimator.updateMeasurements(measurements,
                    new Time(timeSeconds, TimeUnit.SECOND)));

            assertFalse(estimator.isRunning());
            assertEquals(estimator.getLastStateTimestamp(), timeSeconds, 0.0);
            final GNSSEstimation estimation3 = estimator.getEstimation();
            final GNSSKalmanState state3 = estimator.getState();

            assertEquals(estimation1, estimation3);
            assertEquals(state1, state3);

            assertEquals(mUpdateStart, 1);
            assertEquals(mUpdateEnd, 1);
            assertEquals(mPropagateStart, 1);
            assertEquals(mPropagateEnd, 1);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test(expected = NotReadyException.class)
    public void testUpdateWhenNotReadyThrowsNotReadyException()
            throws LockedException, NotReadyException, GNSSException {

        final GNSSKalmanConfig kalmanConfig = generateKalmanConfig();
        final GNSSKalmanFilteredEstimator estimator =
                new GNSSKalmanFilteredEstimator(kalmanConfig);

        estimator.updateMeasurements(Collections.<GNSSMeasurement>emptyList(),
                0.0);
    }

    @Test
    public void testPropagate() throws LockedException, NotReadyException,
            GNSSException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

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
            final int numSatellites = config.getNumberOfSatellites();
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

            final GNSSKalmanConfig kalmanConfig = generateKalmanConfig();
            final GNSSKalmanFilteredEstimator estimator =
                    new GNSSKalmanFilteredEstimator(kalmanConfig, this);

            reset();
            assertEquals(mUpdateStart, 0);
            assertEquals(mUpdateEnd, 0);
            assertEquals(mPropagateStart, 0);
            assertEquals(mPropagateEnd, 0);

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

            assertEquals(estimator.getMeasurements(), measurements);
            assertEquals(estimator.getLastStateTimestamp(), timeSeconds, 0.0);
            final GNSSEstimation estimation1 = estimator.getEstimation();
            assertNotNull(estimation1);

            final GNSSEstimation estimation2 = new GNSSEstimation();
            assertTrue(estimator.getEstimation(estimation2));

            assertEquals(estimation1, estimation2);

            final GNSSKalmanState state1 = estimator.getState();
            assertNotNull(state1);

            final GNSSKalmanState state2 = new GNSSKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            assertEquals(estimation1, state1.getEstimation());

            final ECEFPosition estimatedPosition1 = estimation1.getEcefPosition();
            final ECEFVelocity estimatedVelocity1 = estimation1.getEcefVelocity();

            final double diffX1 = Math.abs(ecefUserPosition.getX()
                    - estimatedPosition1.getX());
            final double diffY1 = Math.abs(ecefUserPosition.getY()
                    - estimatedPosition1.getY());
            final double diffZ1 = Math.abs(ecefUserPosition.getZ()
                    - estimatedPosition1.getZ());
            final double posError1 = Math.max(diffX1, Math.max(diffY1, diffZ1));
            if (posError1 > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition1, POSITION_ERROR));

            final double diffVx1 = Math.abs(ecefUserVelocity.getVx()
                    - estimatedVelocity1.getVx());
            final double diffVy1 = Math.abs(ecefUserVelocity.getVy()
                    - estimatedVelocity1.getVy());
            final double diffVz1 = Math.abs(ecefUserVelocity.getVz()
                    - estimatedVelocity1.getVz());
            final double velError = Math.max(diffVx1, Math.max(diffVy1, diffVz1));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity1, VELOCITY_ERROR));

            assertEquals(mUpdateStart, 1);
            assertEquals(mUpdateEnd, 1);
            assertEquals(mPropagateStart, 1);
            assertEquals(mPropagateEnd, 1);

            // propagate
            assertTrue(estimator.propagate(2.0 * timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(estimator.getLastStateTimestamp(), 2.0 * timeSeconds,
                    0.0);
            final GNSSEstimation estimation3 = estimator.getEstimation();
            final GNSSKalmanState state3 = estimator.getState();

            assertEquals(estimation3, state3.getEstimation());

            final ECEFPosition estimatedPosition3 = estimation3.getEcefPosition();
            final ECEFVelocity estimatedVelocity3 = estimation3.getEcefVelocity();

            final double diffX3 = Math.abs(ecefUserPosition.getX()
                    - estimatedPosition3.getX());
            final double diffY3 = Math.abs(ecefUserPosition.getY()
                    - estimatedPosition3.getY());
            final double diffZ3 = Math.abs(ecefUserPosition.getZ()
                    - estimatedPosition3.getZ());
            final double posError3 = Math.max(diffX3, Math.max(diffY3, diffZ3));
            if (posError3 > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition3, PROPAGATION_ERROR));

            final double diffVx3 = Math.abs(ecefUserVelocity.getVx()
                    - estimatedVelocity3.getVx());
            final double diffVy3 = Math.abs(ecefUserVelocity.getVy()
                    - estimatedVelocity3.getVy());
            final double diffVz3 = Math.abs(ecefUserVelocity.getVz()
                    - estimatedVelocity3.getVz());
            final double velError3 = Math.max(diffVx3, Math.max(diffVy3, diffVz3));
            if (velError3 > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity3, PROPAGATION_ERROR));

            final Matrix covariance1 = state1.getCovariance();
            final Matrix covariance3 = state3.getCovariance();

            final double norm1 = Utils.normF(covariance1);
            final double norm3 = Utils.normF(covariance3);
            assertTrue(norm3 >= norm1);

            assertFalse(estimator.propagate(
                    new Time(2.0 * timeSeconds, TimeUnit.SECOND)));

            assertFalse(estimator.isRunning());
            assertEquals(estimator.getLastStateTimestamp(), 2.0 * timeSeconds,
                    0.0);
            final GNSSEstimation estimation4 = estimator.getEstimation();
            final GNSSKalmanState state4 = estimator.getState();

            assertEquals(estimation3, estimation4);
            assertEquals(state3, state4);

            assertEquals(mUpdateStart, 1);
            assertEquals(mUpdateEnd, 1);
            assertEquals(mPropagateStart, 2);
            assertEquals(mPropagateEnd, 2);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test(expected = NotReadyException.class)
    public void testPropagateWhenNotReadyThrowsNotReadyException()
            throws LockedException, NotReadyException, GNSSException {

        final GNSSKalmanConfig kalmanConfig = generateKalmanConfig();
        final GNSSKalmanFilteredEstimator estimator =
                new GNSSKalmanFilteredEstimator(kalmanConfig);

        estimator.propagate(0.0);
    }

    @Test
    public void testReset() throws LockedException, NotReadyException,
            GNSSException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

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
            final int numSatellites = config.getNumberOfSatellites();
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

            final GNSSKalmanConfig kalmanConfig = generateKalmanConfig();
            final GNSSKalmanFilteredEstimator estimator =
                    new GNSSKalmanFilteredEstimator(kalmanConfig, this);

            reset();
            assertEquals(mUpdateStart, 0);
            assertEquals(mUpdateEnd, 0);
            assertEquals(mPropagateStart, 0);
            assertEquals(mPropagateEnd, 0);

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
            assertEquals(estimator.getMeasurements(), measurements);
            assertEquals(estimator.getLastStateTimestamp(), timeSeconds, 0.0);
            assertEquals(estimator.getLastStateTimestampAsTime(),
                    new Time(timeSeconds, TimeUnit.SECOND));
            final Time timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(timestamp, new Time(timeSeconds, TimeUnit.SECOND));

            final GNSSEstimation estimation1 = estimator.getEstimation();
            assertNotNull(estimation1);

            final GNSSKalmanState state1 = estimator.getState();
            assertNotNull(state1);

            assertEquals(estimation1, state1.getEstimation());

            final ECEFPosition estimatedPosition = estimation1.getEcefPosition();
            final ECEFVelocity estimatedVelocity = estimation1.getEcefVelocity();

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

            // reset
            assertEquals(mReset, 0);

            estimator.reset();

            assertEquals(mReset, 1);
            assertNull(estimator.getMeasurements());
            assertNull(estimator.getEstimation());
            assertNull(estimator.getState());
            assertNull(estimator.getLastStateTimestamp());
            assertFalse(estimator.isRunning());

            // update again with same timestamp now it does make an action
            assertTrue(estimator.updateMeasurements(measurements,
                    new Time(timeSeconds, TimeUnit.SECOND)));

            assertEquals(estimator.getMeasurements(), measurements);
            assertEquals(estimator.getLastStateTimestamp(), timeSeconds, 0.0);
            final GNSSEstimation estimation2 = estimator.getEstimation();
            final GNSSKalmanState state2 = estimator.getState();

            assertEquals(estimation1, estimation2);
            assertEquals(state1, state2);

            assertEquals(mUpdateStart, 2);
            assertEquals(mUpdateEnd, 2);
            assertEquals(mPropagateStart, 2);
            assertEquals(mPropagateEnd, 2);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onUpdateStart(final GNSSKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        mUpdateStart++;
    }

    @Override
    public void onUpdateEnd(final GNSSKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        mUpdateEnd++;
    }

    @Override
    public void onPropagateStart(final GNSSKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        mPropagateStart++;
    }

    @Override
    public void onPropagateEnd(final GNSSKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        mPropagateEnd++;
    }

    @Override
    public void onReset(final GNSSKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        mReset++;
    }

    private void reset() {
        mUpdateStart = 0;
        mUpdateEnd = 0;
        mPropagateStart = 0;
        mPropagateEnd = 0;
        mReset = 0;
    }

    private void checkLocked(final GNSSKalmanFilteredEstimator estimator) {
        assertTrue(estimator.isRunning());
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) { }
        try {
            estimator.setEpochInterval(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) { }
        try {
            estimator.setEpochInterval(new Time(0.0, TimeUnit.SECOND));
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) { }
        try {
            estimator.setConfig(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) { }
        try {
            estimator.updateMeasurements(null, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.updateMeasurements(null,
                    new Time(0.0, TimeUnit.SECOND));
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.propagate(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.propagate(new Time(0.0, TimeUnit.SECOND));
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.reset();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) { }
    }

    private static GNSSKalmanConfig generateKalmanConfig() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialPositionUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockOffsetUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialClockDriftUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double accelerationPSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double clockFrequencyPSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double clockPhasePSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double pseudoRangeSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double rangeRateSD = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);

        return new GNSSKalmanConfig(initialPositionUncertainty,
                initialVelocityUncertainty, initialClockOffsetUncertainty,
                initialClockDriftUncertainty, accelerationPSD, clockFrequencyPSD,
                clockPhasePSD, pseudoRangeSD, rangeRateSD);
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
        final int numberOfSatellites = randomizer.nextInt(MIN_NUM_SAT,
                MAX_NUM_SAT);
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
