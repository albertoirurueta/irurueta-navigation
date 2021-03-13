/*
 * Copyright (C) 2020 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class INSLooselyCoupledKalmanFilteredEstimatorTest implements
        INSLooselyCoupledKalmanFilteredEstimatorListener {

    private static final double MIN_VALUE = 1e-4;
    private static final double MAX_VALUE = 1e-3;

    private static final double MIN_EPOCH_INTERVAL = 1e-5;
    private static final double MAX_EPOCH_INTERVAL = 1.0;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_USER_HEIGHT = -50.0;
    private static final double MAX_USER_HEIGHT = 50.0;

    private static final double MIN_USER_VELOCITY_VALUE = -2.0;
    private static final double MAX_USER_VELOCITY_VALUE = 2.0;

    private static final double MIN_TIME = 0.0;
    private static final double MAX_TIME = 1.0;

    private static final double POSITION_ERROR = 5.0;
    private static final double VELOCITY_ERROR = 5.0;

    private static final double MIN_DEGREES_PER_SECOND = -10.0;
    private static final double MAX_DEGREES_PER_SECOND = 10.0;

    private static final int TIMES = 100;

    private static final double ABSOLUTE_ERROR = 1e-8;

    private int mUpdateStart;
    private int mUpdateEnd;
    private int mPropagateStart;
    private int mPropagateEnd;
    private int mReset;

    @Test
    public void testConstructor()
            throws InvalidSourceAndDestinationFrameTypeException {

        // test constructor 1
        INSLooselyCoupledKalmanFilteredEstimator estimator =
                new INSLooselyCoupledKalmanFilteredEstimator();

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
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        Time lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());


        // test constructor 2
        final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(0.0, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(0.0, TimeUnit.SECOND));
        INSLooselyCoupledKalmanConfig kalmanConfig2 =
                new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(estimator.getConfig(), kalmanConfig);
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());


        // test constructor 3
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double epochInterval = randomizer.nextDouble(
                MIN_EPOCH_INTERVAL, MAX_EPOCH_INTERVAL);
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(-epochInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 4
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(0.0, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(0.0, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());


        // test constructor 5
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                epochInterval);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(estimator.getConfig(), kalmanConfig);
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                    -epochInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 6
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(0.0, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(0.0, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(estimator.getConfig(), kalmanConfig);
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());


        // test constructor 7
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval,
                this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                    -epochInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 8
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                epochInterval, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(estimator.getConfig(), kalmanConfig);
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                    -epochInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 9
        epochIntervalTime = new Time(epochInterval, TimeUnit.SECOND);
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());


        // test constructor 10
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, epochIntervalTime);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(estimator.getConfig(), kalmanConfig);
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        final Time wrongEpochIntervalTime = new Time(-epochInterval, TimeUnit.SECOND);
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                    wrongEpochIntervalTime);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 11
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                epochIntervalTime, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                    wrongEpochIntervalTime, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 12
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                kalmanConfig, epochIntervalTime, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(estimator.getConfig(), kalmanConfig);
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                    wrongEpochIntervalTime, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 13
        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c = new CoordinateTransformation(
                roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

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

        final ECEFFrame frame = new ECEFFrame(ecefUserPosition, ecefUserVelocity, c);

        estimator = new INSLooselyCoupledKalmanFilteredEstimator(frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(0.0, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(0.0, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(estimator.getFrame(), frame);
        ECEFFrame frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());


        // test constructor 14
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(0.0, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(0.0, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(estimator.getConfig(), kalmanConfig);
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());


        // test constructor 15
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                epochInterval, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                    -epochInterval, frame);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 16
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(frame, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(0.0, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(0.0, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());


        // test constructor 17
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                epochInterval, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                    -epochInterval, frame);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 18
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                frame, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(0.0, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(0.0, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(estimator.getConfig(), kalmanConfig);
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());


        // test constructor 19
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval,
                frame, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(-epochInterval,
                    frame, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 20
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                epochInterval, frame, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                    -epochInterval, frame, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 21
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                epochIntervalTime, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                    wrongEpochIntervalTime, frame);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 22
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                epochIntervalTime, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                    wrongEpochIntervalTime, frame);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 23
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochIntervalTime,
                frame, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                    wrongEpochIntervalTime, frame, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 24
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                epochIntervalTime, frame, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        assertFalse(estimator.getInitialConfig(null));
        assertNull(estimator.getInitialConfig());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                    wrongEpochIntervalTime, frame, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 25
        final INSLooselyCoupledKalmanInitializerConfig initialConfig =
                generateInitConfig();
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(0.0, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(0.0, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        INSLooselyCoupledKalmanInitializerConfig initialConfig2 =
                new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());


        // test constructor 26
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(0.0, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(0.0, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());


        // test constructor 27
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval,
                initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                    -epochInterval, initialConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 28
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(initialConfig,
                this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(0.0, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(0.0, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());


        // test constructor 29
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                epochInterval, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(estimator.getConfig(), kalmanConfig);
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                    -epochInterval, initialConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 30
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                initialConfig, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(0.0, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(0.0, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());


        // test constructor 31
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval,
                initialConfig, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                    -epochInterval, initialConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 32
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                epochInterval, initialConfig, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(estimator.getConfig(), kalmanConfig);
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                    -epochInterval, initialConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 33
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                epochIntervalTime, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                    wrongEpochIntervalTime, initialConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 34
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                epochIntervalTime, initialConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(estimator.getConfig(), kalmanConfig);
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                    wrongEpochIntervalTime, initialConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 35
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                epochIntervalTime, initialConfig, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                    wrongEpochIntervalTime, initialConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 36
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                epochIntervalTime, initialConfig, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(estimator.getConfig(), kalmanConfig);
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertSame(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                    wrongEpochIntervalTime, initialConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 37
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(initialConfig,
                frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(0.0, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(0.0, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertEquals(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());


        // test constructor 38
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                initialConfig, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(0.0, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(0.0, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(estimator.getConfig(), kalmanConfig);
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertEquals(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());


        // test constructor 39
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval,
                initialConfig, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                    -epochInterval, initialConfig, frame);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 40
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(initialConfig,
                frame, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(0.0, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(0.0, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertEquals(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());


        // test constructor 41
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                epochInterval, initialConfig, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                    -epochInterval, initialConfig, frame);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 42
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                initialConfig, frame, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);
        epochIntervalTime = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(0.0, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(0.0, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(estimator.getConfig(), kalmanConfig);
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(initialConfig, initialConfig2);
        assertEquals(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());


        // test constructor 43
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(epochInterval,
                initialConfig, frame, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                    -epochInterval, initialConfig, frame, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 44
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                epochInterval, initialConfig, frame, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                    -epochInterval, initialConfig, frame, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 45
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                epochIntervalTime, initialConfig, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                    wrongEpochIntervalTime, initialConfig, frame);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 46
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                epochIntervalTime, initialConfig, frame);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                    wrongEpochIntervalTime, initialConfig, frame);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 47
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                epochIntervalTime, initialConfig, frame, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        assertFalse(estimator.getConfig(null));
        assertNull(estimator.getConfig());
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(
                    wrongEpochIntervalTime, initialConfig, frame, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 48
        estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                epochIntervalTime, initialConfig, frame, this);

        // check default values
        assertSame(estimator.getListener(), this);
        assertEquals(estimator.getEpochInterval(), epochInterval, 0.0);
        epochIntervalTime = new Time(epochInterval, TimeUnit.MILLISECOND);
        estimator.getEpochIntervalAsTime(epochIntervalTime);
        assertEquals(epochIntervalTime, new Time(epochInterval, TimeUnit.SECOND));
        assertEquals(estimator.getEpochIntervalAsTime(),
                new Time(epochInterval, TimeUnit.SECOND));
        kalmanConfig2 = new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(kalmanConfig2));
        assertEquals(kalmanConfig, kalmanConfig2);
        assertEquals(kalmanConfig, estimator.getConfig());
        assertEquals(estimator.getFrame(), frame);
        frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
        initialConfig2 = new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(initialConfig2));
        assertEquals(estimator.getInitialConfig(), initialConfig);
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getLastStateTimestamp());
        lastStateTimestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertFalse(estimator.getLastStateTimestampAsTime(lastStateTimestamp));
        assertNull(estimator.getLastStateTimestampAsTime());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isPropagateReady());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                    wrongEpochIntervalTime, initialConfig, frame, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final INSLooselyCoupledKalmanFilteredEstimator estimator =
                new INSLooselyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testGetSetEpochInterval() throws LockedException {
        final INSLooselyCoupledKalmanFilteredEstimator estimator =
                new INSLooselyCoupledKalmanFilteredEstimator();

        // check default value
        assertEquals(estimator.getEpochInterval(), 0.0, 0.0);

        // set new value
        estimator.setEpochInterval(1.0);

        // check
        assertEquals(estimator.getEpochInterval(), 1.0, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setEpochInterval(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetEpochIntervalAsTime() throws LockedException {
        final INSLooselyCoupledKalmanFilteredEstimator estimator =
                new INSLooselyCoupledKalmanFilteredEstimator();

        final Time epochInterval1 = estimator.getEpochIntervalAsTime();

        assertEquals(epochInterval1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(epochInterval1.getUnit(), TimeUnit.SECOND);

        // set new value
        final Time epochInterval2 = new Time(1.0, TimeUnit.SECOND);
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
        final INSLooselyCoupledKalmanFilteredEstimator estimator =
                new INSLooselyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getConfig());
        assertFalse(estimator.getConfig(null));

        // set new value
        final INSLooselyCoupledKalmanConfig config1 = generateKalmanConfig();
        estimator.setConfig(config1);

        // check
        final INSLooselyCoupledKalmanConfig config2 =
                new INSLooselyCoupledKalmanConfig();
        assertTrue(estimator.getConfig(config2));
        final INSLooselyCoupledKalmanConfig config3 = estimator.getConfig();

        assertEquals(config1, config2);
        assertEquals(config1, config3);
    }

    @Test
    public void testGetSetFrame() throws LockedException,
            InvalidSourceAndDestinationFrameTypeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c = new CoordinateTransformation(
                roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

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

        final ECEFFrame frame = new ECEFFrame(ecefUserPosition, ecefUserVelocity, c);

        final INSLooselyCoupledKalmanFilteredEstimator estimator =
                new INSLooselyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getFrame());
        assertFalse(estimator.getFrame(null));

        // set new value
        estimator.setFrame(frame);

        // check
        assertEquals(frame, estimator.getFrame());

        final ECEFFrame frame2 = new ECEFFrame();
        assertTrue(estimator.getFrame(frame2));
        assertEquals(frame, frame2);
    }

    @Test
    public void testGetSetInitialConfig() throws LockedException {
        final INSLooselyCoupledKalmanFilteredEstimator estimator =
                new INSLooselyCoupledKalmanFilteredEstimator();

        // check default value
        assertNull(estimator.getInitialConfig());
        assertFalse(estimator.getInitialConfig(null));

        // set new value
        final INSLooselyCoupledKalmanInitializerConfig config1 =
                generateInitConfig();
        estimator.setInitialConfig(config1);

        // check
        final INSLooselyCoupledKalmanInitializerConfig config2 =
                new INSLooselyCoupledKalmanInitializerConfig();
        assertTrue(estimator.getInitialConfig(config2));
        final INSLooselyCoupledKalmanInitializerConfig config3 =
                estimator.getInitialConfig();

        assertEquals(config1, config2);
        assertSame(config1, config3);
    }

    @Test
    public void testUpdateWithZeroSpecificForceAndAngularRate()
            throws InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, INSException, InvalidRotationMatrixException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

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

            final double roll = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final CoordinateTransformation c = new CoordinateTransformation(
                    roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final ECEFFrame frame = new ECEFFrame(ecefUserPosition,
                    ecefUserVelocity, c);

            final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
            final INSLooselyCoupledKalmanInitializerConfig initConfig =
                    generateInitConfig();
            final INSLooselyCoupledKalmanFilteredEstimator estimator =
                    new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                            initConfig, frame, this);

            reset();
            assertEquals(mUpdateStart, 0);
            assertEquals(mUpdateEnd, 0);
            assertEquals(mPropagateStart, 0);
            assertEquals(mPropagateEnd, 0);

            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));
            assertNull(estimator.getKinematics());
            assertFalse(estimator.getKinematics(null));
            assertNull(estimator.getCorrectedKinematics());
            assertFalse(estimator.getCorrectedKinematics(null));
            assertTrue(estimator.isUpdateReady());
            assertTrue(estimator.isPropagateReady());

            // update kinematics for the first time, makes no change
            final BodyKinematics kinematics = new BodyKinematics();
            assertTrue(estimator.update(kinematics, timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(estimator.getLastStateTimestamp(), timeSeconds, 0.0);
            assertEquals(estimator.getLastStateTimestampAsTime(),
                    new Time(timeSeconds, TimeUnit.SECOND));
            final Time timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(timestamp, new Time(timeSeconds, TimeUnit.SECOND));
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());

            final INSLooselyCoupledKalmanState state1 = estimator.getState();
            assertNotNull(state1);

            final INSLooselyCoupledKalmanState state2 =
                    new INSLooselyCoupledKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            assertEquals(estimator.getFrame(), state1.getFrame());

            final ECEFPosition estimatedPosition1 = state1.getEcefPosition();
            final ECEFVelocity estimatedVelocity1 = state1.getEcefVelocity();
            final CoordinateTransformation estimatedC1 = state1.getC();

            assertEquals(estimatedPosition1, ecefUserPosition);
            assertEquals(estimatedVelocity1, ecefUserVelocity);
            assertTrue(estimatedC1.equals(c, ABSOLUTE_ERROR));

            // update again with same timestamp makes no action
            assertFalse(estimator.update(kinematics,
                    new Time(timeSeconds, TimeUnit.SECOND)));

            assertFalse(estimator.isRunning());
            assertEquals(estimator.getLastStateTimestamp(), timeSeconds, 0.0);
            final INSLooselyCoupledKalmanState state3 = estimator.getState();

            assertEquals(state1, state3);

            // update with new timestamp adds changes
            assertTrue(estimator.update(kinematics, 2.0 * timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(estimator.getLastStateTimestamp(), 2.0 * timeSeconds,
                    0.0);
            assertEquals(estimator.getLastStateTimestampAsTime(),
                    new Time(2.0 * timeSeconds, TimeUnit.SECOND));
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());

            final INSLooselyCoupledKalmanState state4 = estimator.getState();
            final ECEFPosition estimatedPosition4 = state4.getEcefPosition();
            final ECEFVelocity estimatedVelocity4 = state4.getEcefVelocity();

            final double diffX = Math.abs(ecefUserPosition.getX()
                    - estimatedPosition4.getX());
            final double diffY = Math.abs(ecefUserPosition.getY()
                    - estimatedPosition4.getY());
            final double diffZ = Math.abs(ecefUserPosition.getZ()
                    - estimatedPosition4.getZ());
            final double posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition4, POSITION_ERROR));

            final double diffVx = Math.abs(ecefUserVelocity.getVx()
                    - estimatedVelocity4.getVx());
            final double diffVy = Math.abs(ecefUserVelocity.getVy()
                    - estimatedVelocity4.getVy());
            final double diffVz = Math.abs(ecefUserVelocity.getVz()
                    - estimatedVelocity4.getVz());
            final double velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity4, VELOCITY_ERROR));

            assertEquals(mUpdateStart, 2);
            assertEquals(mUpdateEnd, 2);
            assertEquals(mPropagateStart, 2);
            assertEquals(mPropagateEnd, 2);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testUpdateWithSpecificForceAndZeroAngularRate()
            throws InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, INSException, InvalidRotationMatrixException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

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

            final double roll = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final CoordinateTransformation c = new CoordinateTransformation(
                    roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final ECEFFrame frame = new ECEFFrame(ecefUserPosition,
                    ecefUserVelocity, c);

            final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
            final INSLooselyCoupledKalmanInitializerConfig initConfig =
                    generateInitConfig();
            final INSLooselyCoupledKalmanFilteredEstimator estimator =
                    new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                            initConfig, frame, this);

            reset();
            assertEquals(mUpdateStart, 0);
            assertEquals(mUpdateEnd, 0);
            assertEquals(mPropagateStart, 0);
            assertEquals(mPropagateEnd, 0);

            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));
            assertNull(estimator.getKinematics());
            assertFalse(estimator.getKinematics(null));
            assertNull(estimator.getCorrectedKinematics());
            assertFalse(estimator.getCorrectedKinematics(null));
            assertTrue(estimator.isUpdateReady());
            assertTrue(estimator.isPropagateReady());

            // update kinematics for the first time, makes no change
            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(
                    ecefUserPosition.getX(), ecefUserPosition.getY(),
                    ecefUserPosition.getZ());

            final BodyKinematics kinematics = new BodyKinematics(
                    gravity.getGx(), gravity.getGy(), gravity.getGz(),
                    0.0, 0.0, 0.0);
            assertTrue(estimator.update(kinematics, timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(estimator.getLastStateTimestamp(), timeSeconds, 0.0);
            assertEquals(estimator.getLastStateTimestampAsTime(),
                    new Time(timeSeconds, TimeUnit.SECOND));
            final Time timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(timestamp, new Time(timeSeconds, TimeUnit.SECOND));
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());

            final INSLooselyCoupledKalmanState state1 = estimator.getState();
            assertNotNull(state1);

            final INSLooselyCoupledKalmanState state2 =
                    new INSLooselyCoupledKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            assertEquals(estimator.getFrame(), state1.getFrame());

            final ECEFPosition estimatedPosition1 = state1.getEcefPosition();
            final ECEFVelocity estimatedVelocity1 = state1.getEcefVelocity();
            final CoordinateTransformation estimatedC1 = state1.getC();

            assertEquals(estimatedPosition1, ecefUserPosition);
            assertEquals(estimatedVelocity1, ecefUserVelocity);
            assertTrue(estimatedC1.equals(c, ABSOLUTE_ERROR));

            // update again with same timestamp makes no action
            assertFalse(estimator.update(kinematics,
                    new Time(timeSeconds, TimeUnit.SECOND)));

            assertFalse(estimator.isRunning());
            assertEquals(estimator.getLastStateTimestamp(), timeSeconds, 0.0);
            final INSLooselyCoupledKalmanState state3 = estimator.getState();

            assertEquals(state1, state3);

            // update with new timestamp adds changes
            assertTrue(estimator.update(kinematics, 2.0 * timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(estimator.getLastStateTimestamp(), 2.0 * timeSeconds,
                    0.0);
            assertEquals(estimator.getLastStateTimestampAsTime(),
                    new Time(2.0 * timeSeconds, TimeUnit.SECOND));
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());

            final INSLooselyCoupledKalmanState state4 = estimator.getState();
            final ECEFPosition estimatedPosition4 = state4.getEcefPosition();
            final ECEFVelocity estimatedVelocity4 = state4.getEcefVelocity();

            final double diffX = Math.abs(ecefUserPosition.getX()
                    - estimatedPosition4.getX());
            final double diffY = Math.abs(ecefUserPosition.getY()
                    - estimatedPosition4.getY());
            final double diffZ = Math.abs(ecefUserPosition.getZ()
                    - estimatedPosition4.getZ());
            final double posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition4, POSITION_ERROR));

            final double diffVx = Math.abs(ecefUserVelocity.getVx()
                    - estimatedVelocity4.getVx());
            final double diffVy = Math.abs(ecefUserVelocity.getVy()
                    - estimatedVelocity4.getVy());
            final double diffVz = Math.abs(ecefUserVelocity.getVz()
                    - estimatedVelocity4.getVz());
            final double velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity4, VELOCITY_ERROR));

            assertEquals(mUpdateStart, 2);
            assertEquals(mUpdateEnd, 2);
            assertEquals(mPropagateStart, 2);
            assertEquals(mPropagateEnd, 2);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testUpdateWithZeroSpecificForceAndRotationOnly()
            throws LockedException, NotReadyException, INSException,
            InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

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

            final double roll = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final CoordinateTransformation c = new CoordinateTransformation(
                    roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final ECEFFrame frame = new ECEFFrame(ecefUserPosition,
                    ecefUserVelocity, c);

            final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
            final INSLooselyCoupledKalmanInitializerConfig initConfig =
                    generateInitConfig();
            final INSLooselyCoupledKalmanFilteredEstimator estimator =
                    new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                            initConfig, frame, this);

            reset();
            assertEquals(mUpdateStart, 0);
            assertEquals(mUpdateEnd, 0);
            assertEquals(mPropagateStart, 0);
            assertEquals(mPropagateEnd, 0);

            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));
            assertNull(estimator.getKinematics());
            assertFalse(estimator.getKinematics(null));
            assertNull(estimator.getCorrectedKinematics());
            assertFalse(estimator.getCorrectedKinematics(null));
            assertTrue(estimator.isUpdateReady());
            assertTrue(estimator.isPropagateReady());

            // update kinematics for the first time, makes no change
            final double angularRateX = Math.toRadians(randomizer.nextDouble(
                    MIN_DEGREES_PER_SECOND, MAX_DEGREES_PER_SECOND));
            final double angularRateY = Math.toRadians(randomizer.nextDouble(
                    MIN_DEGREES_PER_SECOND, MAX_DEGREES_PER_SECOND));
            final double angularRateZ = Math.toRadians(randomizer.nextDouble(
                    MIN_DEGREES_PER_SECOND, MAX_DEGREES_PER_SECOND));

            final BodyKinematics kinematics = new BodyKinematics(
                    0.0, 0.0, 0.0,
                    angularRateX, angularRateY, angularRateZ);
            assertTrue(estimator.update(kinematics, timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(estimator.getLastStateTimestamp(), timeSeconds, 0.0);
            assertEquals(estimator.getLastStateTimestampAsTime(),
                    new Time(timeSeconds, TimeUnit.SECOND));
            final Time timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(timestamp, new Time(timeSeconds, TimeUnit.SECOND));
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());

            final INSLooselyCoupledKalmanState state1 = estimator.getState();
            assertNotNull(state1);

            final INSLooselyCoupledKalmanState state2 =
                    new INSLooselyCoupledKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            assertEquals(estimator.getFrame(), state1.getFrame());

            final ECEFPosition estimatedPosition1 = state1.getEcefPosition();
            final ECEFVelocity estimatedVelocity1 = state1.getEcefVelocity();
            final CoordinateTransformation estimatedC1 = state1.getC();

            assertEquals(estimatedPosition1, ecefUserPosition);
            assertEquals(estimatedVelocity1, ecefUserVelocity);
            assertTrue(estimatedC1.equals(c, ABSOLUTE_ERROR));

            // update again with same timestamp makes no action
            assertFalse(estimator.update(kinematics,
                    new Time(timeSeconds, TimeUnit.SECOND)));

            assertFalse(estimator.isRunning());
            assertEquals(estimator.getLastStateTimestamp(), timeSeconds, 0.0);
            final INSLooselyCoupledKalmanState state3 = estimator.getState();

            assertEquals(state1, state3);

            // update with new timestamp adds changes
            assertTrue(estimator.update(kinematics, 2.0 * timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(estimator.getLastStateTimestamp(), 2.0 * timeSeconds,
                    0.0);
            assertEquals(estimator.getLastStateTimestampAsTime(),
                    new Time(2.0 * timeSeconds, TimeUnit.SECOND));
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());

            final INSLooselyCoupledKalmanState state4 = estimator.getState();
            final ECEFPosition estimatedPosition4 = state4.getEcefPosition();
            final ECEFVelocity estimatedVelocity4 = state4.getEcefVelocity();

            final double diffX = Math.abs(ecefUserPosition.getX()
                    - estimatedPosition4.getX());
            final double diffY = Math.abs(ecefUserPosition.getY()
                    - estimatedPosition4.getY());
            final double diffZ = Math.abs(ecefUserPosition.getZ()
                    - estimatedPosition4.getZ());
            final double posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition4, POSITION_ERROR));

            final double diffVx = Math.abs(ecefUserVelocity.getVx()
                    - estimatedVelocity4.getVx());
            final double diffVy = Math.abs(ecefUserVelocity.getVy()
                    - estimatedVelocity4.getVy());
            final double diffVz = Math.abs(ecefUserVelocity.getVz()
                    - estimatedVelocity4.getVz());
            final double velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity4, VELOCITY_ERROR));

            assertEquals(mUpdateStart, 2);
            assertEquals(mUpdateEnd, 2);
            assertEquals(mPropagateStart, 2);
            assertEquals(mPropagateEnd, 2);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testUpdateWithSpecificForceAndRotation()
            throws LockedException, NotReadyException, INSException,
            InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

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

            final double roll = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

            final CoordinateTransformation c = new CoordinateTransformation(
                    roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

            final ECEFFrame frame = new ECEFFrame(ecefUserPosition,
                    ecefUserVelocity, c);

            final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
            final INSLooselyCoupledKalmanInitializerConfig initConfig =
                    generateInitConfig();
            final INSLooselyCoupledKalmanFilteredEstimator estimator =
                    new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                            initConfig, frame, this);

            reset();
            assertEquals(mUpdateStart, 0);
            assertEquals(mUpdateEnd, 0);
            assertEquals(mPropagateStart, 0);
            assertEquals(mPropagateEnd, 0);

            assertNull(estimator.getState());
            assertFalse(estimator.getState(null));
            assertNull(estimator.getKinematics());
            assertFalse(estimator.getKinematics(null));
            assertNull(estimator.getCorrectedKinematics());
            assertFalse(estimator.getCorrectedKinematics(null));
            assertTrue(estimator.isUpdateReady());
            assertTrue(estimator.isPropagateReady());

            // update kinematics for the first time, makes no change
            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(
                    ecefUserPosition.getX(), ecefUserPosition.getY(),
                    ecefUserPosition.getZ());

            final double angularRateX = Math.toRadians(randomizer.nextDouble(
                    MIN_DEGREES_PER_SECOND, MAX_DEGREES_PER_SECOND));
            final double angularRateY = Math.toRadians(randomizer.nextDouble(
                    MIN_DEGREES_PER_SECOND, MAX_DEGREES_PER_SECOND));
            final double angularRateZ = Math.toRadians(randomizer.nextDouble(
                    MIN_DEGREES_PER_SECOND, MAX_DEGREES_PER_SECOND));

            final BodyKinematics kinematics = new BodyKinematics(
                    gravity.getGx(), gravity.getGy(), gravity.getGz(),
                    angularRateX, angularRateY, angularRateZ);
            assertTrue(estimator.update(kinematics, timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(estimator.getLastStateTimestamp(), timeSeconds, 0.0);
            assertEquals(estimator.getLastStateTimestampAsTime(),
                    new Time(timeSeconds, TimeUnit.SECOND));
            final Time timestamp = new Time(0.0, TimeUnit.MILLISECOND);
            assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
            assertEquals(timestamp, new Time(timeSeconds, TimeUnit.SECOND));
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());

            final INSLooselyCoupledKalmanState state1 = estimator.getState();
            assertNotNull(state1);

            final INSLooselyCoupledKalmanState state2 =
                    new INSLooselyCoupledKalmanState();
            assertTrue(estimator.getState(state2));

            assertEquals(state1, state2);

            assertEquals(estimator.getFrame(), state1.getFrame());

            final ECEFPosition estimatedPosition1 = state1.getEcefPosition();
            final ECEFVelocity estimatedVelocity1 = state1.getEcefVelocity();
            final CoordinateTransformation estimatedC1 = state1.getC();

            assertEquals(estimatedPosition1, ecefUserPosition);
            assertEquals(estimatedVelocity1, ecefUserVelocity);
            assertTrue(estimatedC1.equals(c, ABSOLUTE_ERROR));

            // update again with same timestamp makes no action
            assertFalse(estimator.update(kinematics,
                    new Time(timeSeconds, TimeUnit.SECOND)));

            assertFalse(estimator.isRunning());
            assertEquals(estimator.getLastStateTimestamp(), timeSeconds, 0.0);
            final INSLooselyCoupledKalmanState state3 = estimator.getState();

            assertEquals(state1, state3);

            // update with new timestamp adds changes
            assertTrue(estimator.update(kinematics, 2.0 * timeSeconds));

            assertFalse(estimator.isRunning());
            assertEquals(estimator.getLastStateTimestamp(), 2.0 * timeSeconds,
                    0.0);
            assertEquals(estimator.getLastStateTimestampAsTime(),
                    new Time(2.0 * timeSeconds, TimeUnit.SECOND));
            assertEquals(kinematics, estimator.getKinematics());
            assertNotNull(estimator.getCorrectedKinematics());

            final INSLooselyCoupledKalmanState state4 = estimator.getState();
            final ECEFPosition estimatedPosition4 = state4.getEcefPosition();
            final ECEFVelocity estimatedVelocity4 = state4.getEcefVelocity();

            final double diffX = Math.abs(ecefUserPosition.getX()
                    - estimatedPosition4.getX());
            final double diffY = Math.abs(ecefUserPosition.getY()
                    - estimatedPosition4.getY());
            final double diffZ = Math.abs(ecefUserPosition.getZ()
                    - estimatedPosition4.getZ());
            final double posError = Math.max(diffX, Math.max(diffY, diffZ));
            if (posError > POSITION_ERROR) {
                continue;
            }
            assertTrue(ecefUserPosition.equals(estimatedPosition4, POSITION_ERROR));

            final double diffVx = Math.abs(ecefUserVelocity.getVx()
                    - estimatedVelocity4.getVx());
            final double diffVy = Math.abs(ecefUserVelocity.getVy()
                    - estimatedVelocity4.getVy());
            final double diffVz = Math.abs(ecefUserVelocity.getVz()
                    - estimatedVelocity4.getVz());
            final double velError = Math.max(diffVx, Math.max(diffVy, diffVz));
            if (velError > VELOCITY_ERROR) {
                continue;
            }
            assertTrue(ecefUserVelocity.equals(estimatedVelocity4, VELOCITY_ERROR));

            assertEquals(mUpdateStart, 2);
            assertEquals(mUpdateEnd, 2);
            assertEquals(mPropagateStart, 2);
            assertEquals(mPropagateEnd, 2);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test(expected = NotReadyException.class)
    public void testUpdateWhenNotReady() throws LockedException,
            NotReadyException, INSException {

        final INSLooselyCoupledKalmanFilteredEstimator estimator =
                new INSLooselyCoupledKalmanFilteredEstimator();

        assertFalse(estimator.isUpdateReady());

        estimator.update(new BodyKinematics(), 0.0);
    }

    @Test
    public void testPropagate()
            throws InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, INSException, InvalidRotationMatrixException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

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

        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c = new CoordinateTransformation(
                roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final ECEFFrame frame = new ECEFFrame(ecefUserPosition,
                ecefUserVelocity, c);

        final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                generateInitConfig();
        final INSLooselyCoupledKalmanFilteredEstimator estimator =
                new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                        initConfig, frame, this);

        reset();
        assertEquals(mUpdateStart, 0);
        assertEquals(mUpdateEnd, 0);
        assertEquals(mPropagateStart, 0);
        assertEquals(mPropagateEnd, 0);

        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertTrue(estimator.isUpdateReady());
        assertTrue(estimator.isPropagateReady());

        // update kinematics for the first time, makes no change
        final BodyKinematics kinematics = new BodyKinematics();
        assertTrue(estimator.update(kinematics, timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(estimator.getLastStateTimestamp(), timeSeconds, 0.0);
        assertEquals(estimator.getLastStateTimestampAsTime(),
                new Time(timeSeconds, TimeUnit.SECOND));
        final Time timestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
        assertEquals(timestamp, new Time(timeSeconds, TimeUnit.SECOND));
        assertEquals(kinematics, estimator.getKinematics());
        assertNotNull(estimator.getCorrectedKinematics());

        final INSLooselyCoupledKalmanState state1 = estimator.getState();
        assertNotNull(state1);

        final INSLooselyCoupledKalmanState state2 =
                new INSLooselyCoupledKalmanState();
        assertTrue(estimator.getState(state2));

        assertEquals(state1, state2);

        assertEquals(estimator.getFrame(), state1.getFrame());

        final ECEFPosition estimatedPosition1 = state1.getEcefPosition();
        final ECEFVelocity estimatedVelocity1 = state1.getEcefVelocity();
        final CoordinateTransformation estimatedC1 = state1.getC();

        assertEquals(estimatedPosition1, ecefUserPosition);
        assertEquals(estimatedVelocity1, ecefUserVelocity);
        assertTrue(estimatedC1.equals(c, ABSOLUTE_ERROR));

        // propagate
        assertTrue(estimator.propagate(2.0 * timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(estimator.getLastStateTimestamp(), 2.0 * timeSeconds,
                0.0);
        assertEquals(estimator.getLastStateTimestampAsTime(),
                new Time(2.0 * timeSeconds, TimeUnit.SECOND));
        assertEquals(kinematics, estimator.getKinematics());
        assertNotNull(estimator.getCorrectedKinematics());

        final INSLooselyCoupledKalmanState state3 = estimator.getState();
        final ECEFPosition estimatedPosition3 = state3.getEcefPosition();
        final ECEFVelocity estimatedVelocity3 = state3.getEcefVelocity();
        final CoordinateTransformation estimatedC3 = state3.getC();

        assertEquals(estimatedPosition3, ecefUserPosition);
        assertEquals(estimatedVelocity3, ecefUserVelocity);
        assertTrue(estimatedC3.equals(c, ABSOLUTE_ERROR));
        // state is not equals because covariance has changed
        assertNotEquals(state1, state3);

        final Matrix covariance1 = state1.getCovariance();
        final Matrix covariance3 = state3.getCovariance();

        final double norm1 = Utils.normF(covariance1);
        final double norm3 = Utils.normF(covariance3);
        assertTrue(norm3 >= norm1);

        // propagate again with same timestamp has no effect
        assertFalse(estimator.propagate(
                new Time(2.0 * timeSeconds, TimeUnit.SECOND)));

        final INSLooselyCoupledKalmanState state4 = estimator.getState();
        final ECEFPosition estimatedPosition4 = state4.getEcefPosition();
        final ECEFVelocity estimatedVelocity4 = state4.getEcefVelocity();
        final CoordinateTransformation estimatedC4 = state4.getC();

        assertEquals(estimatedPosition4, ecefUserPosition);
        assertEquals(estimatedVelocity4, ecefUserVelocity);
        assertTrue(estimatedC4.equals(c, ABSOLUTE_ERROR));
        assertEquals(state3, state4);

        assertEquals(mUpdateStart, 1);
        assertEquals(mUpdateEnd, 1);
        assertEquals(mPropagateStart, 2);
        assertEquals(mPropagateEnd, 2);
    }

    @Test(expected = NotReadyException.class)
    public void testPropagateWhenNotReady() throws LockedException,
            NotReadyException, INSException {

        final INSLooselyCoupledKalmanFilteredEstimator estimator =
                new INSLooselyCoupledKalmanFilteredEstimator();

        assertFalse(estimator.isPropagateReady());

        estimator.propagate(0.0);
    }

    @Test
    public void testReset() throws InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, INSException, InvalidRotationMatrixException {


        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

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

        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        final CoordinateTransformation c = new CoordinateTransformation(
                roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);

        final ECEFFrame frame = new ECEFFrame(ecefUserPosition,
                ecefUserVelocity, c);

        final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                generateInitConfig();
        final INSLooselyCoupledKalmanFilteredEstimator estimator =
                new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                        initConfig, frame, this);

        reset();
        assertEquals(mUpdateStart, 0);
        assertEquals(mUpdateEnd, 0);
        assertEquals(mPropagateStart, 0);
        assertEquals(mPropagateEnd, 0);

        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
        assertNull(estimator.getKinematics());
        assertFalse(estimator.getKinematics(null));
        assertNull(estimator.getCorrectedKinematics());
        assertFalse(estimator.getCorrectedKinematics(null));
        assertTrue(estimator.isUpdateReady());
        assertTrue(estimator.isPropagateReady());

        // update kinematics for the first time, makes no change
        final BodyKinematics kinematics = new BodyKinematics();
        assertTrue(estimator.update(kinematics, timeSeconds));

        assertFalse(estimator.isRunning());
        assertEquals(estimator.getLastStateTimestamp(), timeSeconds, 0.0);
        assertEquals(estimator.getLastStateTimestampAsTime(),
                new Time(timeSeconds, TimeUnit.SECOND));
        final Time timestamp = new Time(0.0, TimeUnit.MILLISECOND);
        assertTrue(estimator.getLastStateTimestampAsTime(timestamp));
        assertEquals(timestamp, new Time(timeSeconds, TimeUnit.SECOND));
        assertEquals(kinematics, estimator.getKinematics());
        assertNotNull(estimator.getCorrectedKinematics());

        final INSLooselyCoupledKalmanState state1 = estimator.getState();
        assertNotNull(state1);

        assertEquals(estimator.getFrame(), state1.getFrame());

        final ECEFPosition estimatedPosition1 = state1.getEcefPosition();
        final ECEFVelocity estimatedVelocity1 = state1.getEcefVelocity();
        final CoordinateTransformation estimatedC1 = state1.getC();

        assertEquals(estimatedPosition1, ecefUserPosition);
        assertEquals(estimatedVelocity1, ecefUserVelocity);
        assertTrue(estimatedC1.equals(c, ABSOLUTE_ERROR));

        // reset
        assertEquals(mReset, 0);

        estimator.reset();

        assertEquals(mReset, 1);
        assertNull(estimator.getState());
        assertNull(estimator.getLastStateTimestamp());
        assertNull(estimator.getFrame());
        assertNull(estimator.getKinematics());
        assertFalse(estimator.isRunning());

        assertFalse(estimator.isUpdateReady());

        // set frame
        estimator.setFrame(frame);

        assertTrue(estimator.isUpdateReady());

        // update again with same timestamp now it does make an action
        assertTrue(estimator.update(kinematics,
                new Time(timeSeconds, TimeUnit.SECOND)));

        assertEquals(estimator.getLastStateTimestamp(), timeSeconds, 0.0);
        final INSLooselyCoupledKalmanState state2 = estimator.getState();

        assertTrue(state1.equals(state2, ABSOLUTE_ERROR));

        assertEquals(mUpdateStart, 2);
        assertEquals(mUpdateEnd, 2);
        assertEquals(mPropagateStart, 2);
        assertEquals(mPropagateEnd, 2);
    }

    @Override
    public void onUpdateStart(
            final INSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        mUpdateStart++;
    }

    @Override
    public void onUpdateEnd(
            final INSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        mUpdateEnd++;
    }

    @Override
    public void onPropagateStart(
            final INSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        mPropagateStart++;
    }

    @Override
    public void onPropagateEnd(
            final INSLooselyCoupledKalmanFilteredEstimator estimator) {
        checkLocked(estimator);
        mPropagateEnd++;
    }

    @Override
    public void onReset(
            final INSLooselyCoupledKalmanFilteredEstimator estimator) {
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

    private void checkLocked(
            final INSLooselyCoupledKalmanFilteredEstimator estimator) {
        assertTrue(estimator.isRunning());
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setEpochInterval(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setEpochInterval(new Time(0.0, TimeUnit.SECOND));
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setConfig(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setFrame(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setInitialConfig(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.update(null, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.update(null,
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
        } catch (final LockedException ignore) {
        }
    }

    private static INSLooselyCoupledKalmanInitializerConfig generateInitConfig() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double initialAttitudeUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialVelocityUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialPositionUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialAccelerationBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);
        final double initialGyroscopeBiasUncertainty = randomizer.nextDouble(
                MIN_VALUE, MAX_VALUE);

        return new INSLooselyCoupledKalmanInitializerConfig(
                initialAttitudeUncertainty, initialVelocityUncertainty,
                initialPositionUncertainty, initialAccelerationBiasUncertainty,
                initialGyroscopeBiasUncertainty);
    }

    private static INSLooselyCoupledKalmanConfig generateKalmanConfig() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double gyroNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerNoisePSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double accelerometerBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double gyroBiasPSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double positionNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);
        final double velocityNoiseSD = randomizer.nextDouble(MIN_VALUE, MAX_VALUE);

        return new INSLooselyCoupledKalmanConfig(gyroNoisePSD, accelerometerNoisePSD,
                accelerometerBiasPSD, gyroBiasPSD, positionNoiseSD, velocityNoiseSD);
    }
}
