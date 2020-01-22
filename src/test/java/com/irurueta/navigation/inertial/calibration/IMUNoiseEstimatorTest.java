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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class IMUNoiseEstimatorTest implements IMUNoiseEstimatorListener {

    private static final double MIN_ACCELEROMETER_VALUE = -2.0 * 9.81;
    private static final double MAX_ACCELEROMETER_VALUE = 2.0 * 9.81;

    private static final double MIN_GYRO_VALUE = -2.0;
    private static final double MAX_GYRO_VALUE = 2.0;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double ABSOLUTE_ERROR = 1e-5;
    private static final double SMALL_ABSOLUTE_ERROR = 1e-12;
    private static final double LARGE_ABSOLUTE_ERROR = 0.1;

    private int mStart;
    private int mBodyKinematicsAdded;
    private int mFinish;
    private int mReset;

    @Test
    public void testConstructor() {
        final Acceleration acceleration1 = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AngularSpeed angularSpeed1 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final BodyKinematics kinematics1 = new BodyKinematics();
        final BodyKinematics kinematics2 = new BodyKinematics();
        final Time time1 = new Time(IMUNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.SECOND);


        // test constructor 1
        IMUNoiseEstimator estimator = new IMUNoiseEstimator();

        // check default values
        assertEquals(estimator.getTotalSamples(), IMUNoiseEstimator.DEFAULT_TOTAL_SAMPLES);
        assertEquals(estimator.getTimeInterval(),
                IMUNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 0.0);
        assertEquals(estimator.getTimeIntervalAsTime(), time1);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(estimator.getAvgFx(), 0.0, 0.0);
        assertEquals(estimator.getAvgFxAsAcceleration(), acceleration1);
        estimator.getAvgFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFy(), 0.0, 0.0);
        assertEquals(estimator.getAvgFyAsAcceleration(), acceleration1);
        estimator.getAvgFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFz(), 0.0, 0.0);
        assertEquals(estimator.getAvgFzAsAcceleration(), acceleration1);
        estimator.getAvgFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgBodyKinematics(), new BodyKinematics());
        assertEquals(estimator.getVarianceFx(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFy(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFz(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFx(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFxAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFy(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFyAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFz(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFzAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAverageAccelerometerStandardDeviation(), 0.0,
                0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(),
                acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationAngularRateX(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateY(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateZ(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAverageGyroscopeStandardDeviation(), 0.0, 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(),
                angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationsAsBodyKinematics(), kinematics1);
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(estimator.getPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());


        // test constructor2
        estimator = new IMUNoiseEstimator(this);

        // check default values
        assertEquals(estimator.getTotalSamples(), IMUNoiseEstimator.DEFAULT_TOTAL_SAMPLES);
        assertEquals(estimator.getTimeInterval(),
                IMUNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 0.0);
        assertEquals(estimator.getTimeIntervalAsTime(), time1);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(estimator.getAvgFx(), 0.0, 0.0);
        assertEquals(estimator.getAvgFxAsAcceleration(), acceleration1);
        estimator.getAvgFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFy(), 0.0, 0.0);
        assertEquals(estimator.getAvgFyAsAcceleration(), acceleration1);
        estimator.getAvgFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFz(), 0.0, 0.0);
        assertEquals(estimator.getAvgFzAsAcceleration(), acceleration1);
        estimator.getAvgFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgBodyKinematics(), new BodyKinematics());
        assertEquals(estimator.getVarianceFx(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFy(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFz(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFx(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFxAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFy(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFyAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFz(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFzAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAverageAccelerometerStandardDeviation(), 0.0,
                0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(),
                acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationAngularRateX(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateY(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateZ(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAverageGyroscopeStandardDeviation(), 0.0, 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(),
                angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationsAsBodyKinematics(), kinematics1);
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(estimator.getPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());


        // test constructor 3
        estimator = new IMUNoiseEstimator(1);

        // check default values
        assertEquals(estimator.getTotalSamples(), 1);
        assertEquals(estimator.getTimeInterval(),
                IMUNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 0.0);
        assertEquals(estimator.getTimeIntervalAsTime(), time1);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(estimator.getAvgFx(), 0.0, 0.0);
        assertEquals(estimator.getAvgFxAsAcceleration(), acceleration1);
        estimator.getAvgFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFy(), 0.0, 0.0);
        assertEquals(estimator.getAvgFyAsAcceleration(), acceleration1);
        estimator.getAvgFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFz(), 0.0, 0.0);
        assertEquals(estimator.getAvgFzAsAcceleration(), acceleration1);
        estimator.getAvgFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgBodyKinematics(), new BodyKinematics());
        assertEquals(estimator.getVarianceFx(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFy(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFz(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFx(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFxAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFy(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFyAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFz(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFzAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAverageAccelerometerStandardDeviation(), 0.0,
                0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(),
                acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationAngularRateX(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateY(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateZ(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAverageGyroscopeStandardDeviation(), 0.0, 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(),
                angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationsAsBodyKinematics(), kinematics1);
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(estimator.getPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new IMUNoiseEstimator(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 4
        estimator = new IMUNoiseEstimator(1, this);

        // check default values
        assertEquals(estimator.getTotalSamples(), 1);
        assertEquals(estimator.getTimeInterval(),
                IMUNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 0.0);
        assertEquals(estimator.getTimeIntervalAsTime(), time1);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(estimator.getAvgFx(), 0.0, 0.0);
        assertEquals(estimator.getAvgFxAsAcceleration(), acceleration1);
        estimator.getAvgFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFy(), 0.0, 0.0);
        assertEquals(estimator.getAvgFyAsAcceleration(), acceleration1);
        estimator.getAvgFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFz(), 0.0, 0.0);
        assertEquals(estimator.getAvgFzAsAcceleration(), acceleration1);
        estimator.getAvgFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgBodyKinematics(), new BodyKinematics());
        assertEquals(estimator.getVarianceFx(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFy(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFz(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFx(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFxAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFy(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFyAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFz(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFzAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAverageAccelerometerStandardDeviation(), 0.0,
                0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(),
                acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationAngularRateX(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateY(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateZ(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAverageGyroscopeStandardDeviation(), 0.0, 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(),
                angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationsAsBodyKinematics(), kinematics1);
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(estimator.getPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new IMUNoiseEstimator(0, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 5
        estimator = new IMUNoiseEstimator(1.0);

        // check default values
        assertEquals(estimator.getTotalSamples(), IMUNoiseEstimator.DEFAULT_TOTAL_SAMPLES);
        assertEquals(estimator.getTimeInterval(), 1.0, 0.0);
        time1.setValue(1.0);
        assertEquals(estimator.getTimeIntervalAsTime(), time1);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(estimator.getAvgFx(), 0.0, 0.0);
        assertEquals(estimator.getAvgFxAsAcceleration(), acceleration1);
        estimator.getAvgFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFy(), 0.0, 0.0);
        assertEquals(estimator.getAvgFyAsAcceleration(), acceleration1);
        estimator.getAvgFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFz(), 0.0, 0.0);
        assertEquals(estimator.getAvgFzAsAcceleration(), acceleration1);
        estimator.getAvgFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgBodyKinematics(), new BodyKinematics());
        assertEquals(estimator.getVarianceFx(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFy(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFz(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFx(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFxAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFy(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFyAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFz(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFzAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAverageAccelerometerStandardDeviation(), 0.0,
                0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(),
                acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationAngularRateX(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateY(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateZ(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAverageGyroscopeStandardDeviation(), 0.0, 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(),
                angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationsAsBodyKinematics(), kinematics1);
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(estimator.getPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new IMUNoiseEstimator(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 6
        final Time timeInterval = new Time(1.0, TimeUnit.SECOND);
        estimator = new IMUNoiseEstimator(timeInterval);

        // check default values
        assertEquals(estimator.getTotalSamples(), IMUNoiseEstimator.DEFAULT_TOTAL_SAMPLES);
        assertEquals(estimator.getTimeInterval(),
                timeInterval.getValue().doubleValue(), 0.0);
        assertEquals(estimator.getTimeIntervalAsTime(), timeInterval);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(timeInterval, time2);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(estimator.getAvgFx(), 0.0, 0.0);
        assertEquals(estimator.getAvgFxAsAcceleration(), acceleration1);
        estimator.getAvgFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFy(), 0.0, 0.0);
        assertEquals(estimator.getAvgFyAsAcceleration(), acceleration1);
        estimator.getAvgFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFz(), 0.0, 0.0);
        assertEquals(estimator.getAvgFzAsAcceleration(), acceleration1);
        estimator.getAvgFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgBodyKinematics(), new BodyKinematics());
        assertEquals(estimator.getVarianceFx(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFy(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFz(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFx(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFxAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFy(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFyAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFz(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFzAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAverageAccelerometerStandardDeviation(), 0.0,
                0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(),
                acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationAngularRateX(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateY(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateZ(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAverageGyroscopeStandardDeviation(), 0.0, 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(),
                angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationsAsBodyKinematics(), kinematics1);
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(estimator.getPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new IMUNoiseEstimator(new Time(-1.0, TimeUnit.SECOND));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 7
        estimator = new IMUNoiseEstimator(1.0, this);

        // check default values
        assertEquals(estimator.getTotalSamples(), IMUNoiseEstimator.DEFAULT_TOTAL_SAMPLES);
        assertEquals(estimator.getTimeInterval(), 1.0, 0.0);
        time1.setValue(1.0);
        assertEquals(estimator.getTimeIntervalAsTime(), time1);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(estimator.getAvgFx(), 0.0, 0.0);
        assertEquals(estimator.getAvgFxAsAcceleration(), acceleration1);
        estimator.getAvgFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFy(), 0.0, 0.0);
        assertEquals(estimator.getAvgFyAsAcceleration(), acceleration1);
        estimator.getAvgFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFz(), 0.0, 0.0);
        assertEquals(estimator.getAvgFzAsAcceleration(), acceleration1);
        estimator.getAvgFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgBodyKinematics(), new BodyKinematics());
        assertEquals(estimator.getVarianceFx(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFy(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFz(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFx(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFxAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFy(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFyAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFz(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFzAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAverageAccelerometerStandardDeviation(), 0.0,
                0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(),
                acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationAngularRateX(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateY(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateZ(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAverageGyroscopeStandardDeviation(), 0.0, 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(),
                angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationsAsBodyKinematics(), kinematics1);
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(estimator.getPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new IMUNoiseEstimator(-1.0, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 8
        estimator = new IMUNoiseEstimator(timeInterval, this);

        // check default values
        assertEquals(estimator.getTotalSamples(), IMUNoiseEstimator.DEFAULT_TOTAL_SAMPLES);
        assertEquals(estimator.getTimeInterval(),
                timeInterval.getValue().doubleValue(), 0.0);
        assertEquals(estimator.getTimeIntervalAsTime(), timeInterval);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(timeInterval, time2);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(estimator.getAvgFx(), 0.0, 0.0);
        assertEquals(estimator.getAvgFxAsAcceleration(), acceleration1);
        estimator.getAvgFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFy(), 0.0, 0.0);
        assertEquals(estimator.getAvgFyAsAcceleration(), acceleration1);
        estimator.getAvgFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFz(), 0.0, 0.0);
        assertEquals(estimator.getAvgFzAsAcceleration(), acceleration1);
        estimator.getAvgFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgBodyKinematics(), new BodyKinematics());
        assertEquals(estimator.getVarianceFx(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFy(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFz(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFx(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFxAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFy(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFyAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFz(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFzAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAverageAccelerometerStandardDeviation(), 0.0,
                0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(),
                acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationAngularRateX(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateY(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateZ(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAverageGyroscopeStandardDeviation(), 0.0, 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(),
                angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationsAsBodyKinematics(), kinematics1);
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(estimator.getPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new IMUNoiseEstimator(new Time(-1.0, TimeUnit.SECOND),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 9
        estimator = new IMUNoiseEstimator(2, 1.0);

        // check default values
        assertEquals(estimator.getTotalSamples(), 2);
        assertEquals(estimator.getTimeInterval(), 1.0, 0.0);
        time1.setValue(1.0);
        assertEquals(estimator.getTimeIntervalAsTime(), time1);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(estimator.getAvgFx(), 0.0, 0.0);
        assertEquals(estimator.getAvgFxAsAcceleration(), acceleration1);
        estimator.getAvgFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFy(), 0.0, 0.0);
        assertEquals(estimator.getAvgFyAsAcceleration(), acceleration1);
        estimator.getAvgFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFz(), 0.0, 0.0);
        assertEquals(estimator.getAvgFzAsAcceleration(), acceleration1);
        estimator.getAvgFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgBodyKinematics(), new BodyKinematics());
        assertEquals(estimator.getVarianceFx(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFy(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFz(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFx(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFxAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFy(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFyAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFz(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFzAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAverageAccelerometerStandardDeviation(), 0.0,
                0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(),
                acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationAngularRateX(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateY(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateZ(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAverageGyroscopeStandardDeviation(), 0.0, 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(),
                angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationsAsBodyKinematics(), kinematics1);
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(estimator.getPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new IMUNoiseEstimator(0, 1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new IMUNoiseEstimator(1, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 10
        estimator = new IMUNoiseEstimator(2, timeInterval);

        // check default values
        assertEquals(estimator.getTotalSamples(), 2);
        assertEquals(estimator.getTimeInterval(), timeInterval.getValue().doubleValue(),
                0.0);
        assertEquals(estimator.getTimeIntervalAsTime(), timeInterval);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(timeInterval, time2);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(estimator.getAvgFx(), 0.0, 0.0);
        assertEquals(estimator.getAvgFxAsAcceleration(), acceleration1);
        estimator.getAvgFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFy(), 0.0, 0.0);
        assertEquals(estimator.getAvgFyAsAcceleration(), acceleration1);
        estimator.getAvgFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFz(), 0.0, 0.0);
        assertEquals(estimator.getAvgFzAsAcceleration(), acceleration1);
        estimator.getAvgFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgBodyKinematics(), new BodyKinematics());
        assertEquals(estimator.getVarianceFx(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFy(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFz(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFx(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFxAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFy(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFyAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFz(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFzAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAverageAccelerometerStandardDeviation(), 0.0,
                0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(),
                acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationAngularRateX(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateY(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateZ(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAverageGyroscopeStandardDeviation(), 0.0, 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(),
                angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationsAsBodyKinematics(), kinematics1);
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(estimator.getPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new IMUNoiseEstimator(0, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new IMUNoiseEstimator(1,
                    new Time(-1.0, TimeUnit.SECOND));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 11
        estimator = new IMUNoiseEstimator(2, 1.0,
                this);

        // check default values
        assertEquals(estimator.getTotalSamples(), 2);
        assertEquals(estimator.getTimeInterval(), 1.0, 0.0);
        time1.setValue(1.0);
        assertEquals(estimator.getTimeIntervalAsTime(), time1);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(estimator.getAvgFx(), 0.0, 0.0);
        assertEquals(estimator.getAvgFxAsAcceleration(), acceleration1);
        estimator.getAvgFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFy(), 0.0, 0.0);
        assertEquals(estimator.getAvgFyAsAcceleration(), acceleration1);
        estimator.getAvgFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFz(), 0.0, 0.0);
        assertEquals(estimator.getAvgFzAsAcceleration(), acceleration1);
        estimator.getAvgFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgBodyKinematics(), new BodyKinematics());
        assertEquals(estimator.getVarianceFx(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFy(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFz(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFx(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFxAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFy(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFyAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFz(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFzAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAverageAccelerometerStandardDeviation(), 0.0,
                0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(),
                acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationAngularRateX(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateY(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateZ(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAverageGyroscopeStandardDeviation(), 0.0, 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(),
                angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationsAsBodyKinematics(), kinematics1);
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(estimator.getPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new IMUNoiseEstimator(0, 1.0,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new IMUNoiseEstimator(1, -1.0,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);


        // test constructor 12
        estimator = new IMUNoiseEstimator(2, timeInterval, this);

        // check default values
        assertEquals(estimator.getTotalSamples(), 2);
        assertEquals(estimator.getTimeInterval(), timeInterval.getValue().doubleValue(),
                0.0);
        assertEquals(estimator.getTimeIntervalAsTime(), timeInterval);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(timeInterval, time2);
        assertSame(estimator.getListener(), this);
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(estimator.getAvgFx(), 0.0, 0.0);
        assertEquals(estimator.getAvgFxAsAcceleration(), acceleration1);
        estimator.getAvgFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFy(), 0.0, 0.0);
        assertEquals(estimator.getAvgFyAsAcceleration(), acceleration1);
        estimator.getAvgFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgFz(), 0.0, 0.0);
        assertEquals(estimator.getAvgFzAsAcceleration(), acceleration1);
        estimator.getAvgFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAvgAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateXAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateYAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAvgAngularRateZAsAngularSpeed(), angularSpeed1);
        estimator.getAvgAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAvgBodyKinematics(), new BodyKinematics());
        assertEquals(estimator.getVarianceFx(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFy(), 0.0, 0.0);
        assertEquals(estimator.getVarianceFz(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getVarianceAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFx(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFxAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFxAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFy(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFyAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFyAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationFz(), 0.0, 0.0);
        assertEquals(estimator.getStandardDeviationFzAsAcceleration(), acceleration1);
        estimator.getStandardDeviationFzAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getAverageAccelerometerStandardDeviation(), 0.0,
                0.0);
        assertEquals(estimator.getAverageAccelerometerStandardDeviationAsAcceleration(),
                acceleration1);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(estimator.getStandardDeviationAngularRateX(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateXAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateY(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateYAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationAngularRateZ(),
                0.0, 0.0);
        assertEquals(estimator.getStandardDeviationAngularRateZAsAngularSpeed(),
                angularSpeed1);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getAverageGyroscopeStandardDeviation(), 0.0, 0.0);
        assertEquals(estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(),
                angularSpeed1);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(estimator.getStandardDeviationsAsBodyKinematics(), kinematics1);
        estimator.getStandardDeviationsAsBodyKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
        assertEquals(estimator.getPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFx(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFy(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDFz(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateX(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateY(), 0.0, 0.0);
        assertEquals(estimator.getRootPSDAngularRateZ(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getAccelerometerNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoisePSD(), 0.0, 0.0);
        assertEquals(estimator.getGyroNoiseRootPSD(), 0.0, 0.0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFinished());

        // Force IllegalArgumentException
        estimator = null;
        try {
            estimator = new IMUNoiseEstimator(0, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new IMUNoiseEstimator(1,
                    new Time(-1.0, TimeUnit.SECOND), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetTotalSamples() throws LockedException {
        final IMUNoiseEstimator estimator = new IMUNoiseEstimator();

        // check default value
        assertEquals(estimator.getTotalSamples(),
                IMUNoiseEstimator.DEFAULT_TOTAL_SAMPLES);

        // set new value
        estimator.setTotalSamples(1);

        // check
        assertEquals(estimator.getTotalSamples(), 1);

        // Force IllegalArgumentException
        try {
            estimator.setTotalSamples(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetTimeInterval() throws LockedException {
        final IMUNoiseEstimator estimator = new IMUNoiseEstimator();

        // check default value
        assertEquals(estimator.getTimeInterval(),
                IMUNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 0.0);

        // set new value
        estimator.setTimeInterval(2.0);

        // check
        assertEquals(estimator.getTimeInterval(), 2.0, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setTimeInterval(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetTimeIntervalAsTime() throws LockedException {
        final IMUNoiseEstimator estimator = new IMUNoiseEstimator();

        // check default value
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();

        assertEquals(timeInterval1.getValue().doubleValue(),
                IMUNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, 0.0);
        assertEquals(timeInterval1.getUnit(), TimeUnit.SECOND);

        // set new value
        final Time timeInterval2 = new Time(2.0, TimeUnit.SECOND);
        estimator.setTimeInterval(timeInterval2);

        // check
        final Time timeInterval3 = estimator.getTimeIntervalAsTime();
        final Time timeInterval4 = new Time(0.0, TimeUnit.MILLISECOND);
        estimator.getTimeIntervalAsTime(timeInterval4);

        assertEquals(timeInterval2, timeInterval3);
        assertEquals(timeInterval2, timeInterval4);

        // Force IllegalArgumentException
        try {
            estimator.setTimeInterval(new Time(-1.0, TimeUnit.SECOND));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final IMUNoiseEstimator estimator = new IMUNoiseEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(estimator.getListener(), this);
    }

    @Test
    public void testAddBodyKinematicsAndReset()
            throws WrongSizeException, LockedException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);
        final double fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final BodyKinematics trueKinematics = new BodyKinematics(fx, fy, fz,
                omegaX, omegaY, omegaZ);


        final IMUNoiseEstimator estimator = new IMUNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mBodyKinematicsAdded, 0);
        assertEquals(mFinish, 0);
        assertEquals(mReset, 0);
        assertFalse(estimator.isFinished());
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.isRunning());

        final BodyKinematics kinematics = new BodyKinematics();
        final int totalSamples = estimator.getTotalSamples();
        final double timeInterval = estimator.getTimeInterval();
        BodyKinematics lastKinematics = new BodyKinematics();
        double avgFx = 0.0;
        double avgFy = 0.0;
        double avgFz = 0.0;
        double avgOmegaX = 0.0;
        double avgOmegaY = 0.0;
        double avgOmegaZ = 0.0;
        double varFx = 0.0;
        double varFy = 0.0;
        double varFz = 0.0;
        double varOmegaX = 0.0;
        double varOmegaY = 0.0;
        double varOmegaZ = 0.0;
        for (int i = 0, j = 1; i < totalSamples; i++, j++) {
            if (estimator.getLastBodyKinematics(lastKinematics)) {
                assertEquals(estimator.getLastBodyKinematics(), lastKinematics);
                assertEquals(lastKinematics, kinematics);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                    errors, random, kinematics);

            assertTrue(estimator.addBodyKinematics(kinematics));

            assertTrue(estimator.getLastBodyKinematics(lastKinematics));
            assertEquals(lastKinematics, kinematics);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avgFx = avgFx * (double) i / (double) j + kinematics.getFx() / j;
            avgFy = avgFy * (double) i / (double) j + kinematics.getFy() / j;
            avgFz = avgFz * (double) i / (double) j + kinematics.getFz() / j;

            avgOmegaX = avgOmegaX * (double) i / (double) j
                    + kinematics.getAngularRateX() / j;
            avgOmegaY = avgOmegaY * (double) i / (double) j
                    + kinematics.getAngularRateY() / j;
            avgOmegaZ = avgOmegaZ * (double) i / (double) j
                    + kinematics.getAngularRateZ() / j;

            double diff = kinematics.getFx() - avgFx;
            double diff2 = diff * diff;
            varFx = varFx * (double) i / (double) j + diff2 / j;

            diff = kinematics.getFy() - avgFy;
            diff2 = diff * diff;
            varFy = varFy * (double) i / (double) j + diff2 / j;

            diff = kinematics.getFz() - avgFz;
            diff2 = diff * diff;
            varFz = varFz * (double) i / (double) j + diff2 / j;

            diff = kinematics.getAngularRateX() - avgOmegaX;
            diff2 = diff * diff;
            varOmegaX = varOmegaX * (double) i / (double) j + diff2 / j;

            diff = kinematics.getAngularRateY() - avgOmegaY;
            diff2 = diff * diff;
            varOmegaY = varOmegaY * (double) i / (double) j + diff2 / j;

            diff = kinematics.getAngularRateZ() - avgOmegaZ;
            diff2 = diff * diff;
            varOmegaZ = varOmegaZ * (double) i / (double) j + diff2 / j;
        }

        assertEquals(estimator.getNumberOfProcessedSamples(), totalSamples);
        assertTrue(estimator.isFinished());
        assertFalse(estimator.isRunning());
        assertEquals(mStart, 1);
        assertEquals(mBodyKinematicsAdded, totalSamples);
        assertEquals(mFinish, 1);
        assertEquals(mReset, 0);

        final double avgFxB = estimator.getAvgFx();
        final double avgFyB = estimator.getAvgFy();
        final double avgFzB = estimator.getAvgFz();

        final double avgAngularRateXB = estimator.getAvgAngularRateX();
        final double avgAngularRateYB = estimator.getAvgAngularRateY();
        final double avgAngularRateZB = estimator.getAvgAngularRateZ();

        assertEquals(avgFx, avgFxB, 0.0);
        assertEquals(avgFy, avgFyB, 0.0);
        assertEquals(avgFz, avgFzB, 0.0);
        assertEquals(avgOmegaX, avgAngularRateXB, 0.0);
        assertEquals(avgOmegaY, avgAngularRateYB, 0.0);
        assertEquals(avgOmegaZ, avgAngularRateZB, 0.0);

        assertEquals(avgFxB, fx, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgFyB, fy, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgFzB, fz, LARGE_ABSOLUTE_ERROR);

        assertEquals(avgAngularRateXB, omegaX, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgAngularRateYB, omegaY, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgAngularRateZB, omegaZ, LARGE_ABSOLUTE_ERROR);

        final Acceleration avgFx1 = estimator.getAvgFxAsAcceleration();
        final Acceleration avgFx2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgFxAsAcceleration(avgFx2);

        assertEquals(avgFx1.getValue().doubleValue(), avgFxB, 0.0);
        assertEquals(avgFx1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(avgFx1, avgFx2);

        final Acceleration avgFy1 = estimator.getAvgFyAsAcceleration();
        final Acceleration avgFy2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgFyAsAcceleration(avgFy2);

        assertEquals(avgFy1.getValue().doubleValue(), avgFyB, 0.0);
        assertEquals(avgFy1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(avgFy1, avgFy2);

        final Acceleration avgFz1 = estimator.getAvgFzAsAcceleration();
        final Acceleration avgFz2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgFzAsAcceleration(avgFz2);

        assertEquals(avgFz1.getValue().doubleValue(), avgFzB, 0.0);
        assertEquals(avgFz1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(avgFz1, avgFz2);

        final AngularSpeed avgAngularRateX1 = estimator
                .getAvgAngularRateXAsAngularSpeed();
        final AngularSpeed avgAngularRateX2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAvgAngularRateXAsAngularSpeed(avgAngularRateX2);

        assertEquals(avgAngularRateX1.getValue().doubleValue(),
                avgAngularRateXB, 0.0);
        assertEquals(avgAngularRateX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(avgAngularRateX1, avgAngularRateX2);

        final AngularSpeed avgAngularRateY1 = estimator
                .getAvgAngularRateYAsAngularSpeed();
        final AngularSpeed avgAngularRateY2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAvgAngularRateYAsAngularSpeed(avgAngularRateY2);

        assertEquals(avgAngularRateY1.getValue().doubleValue(),
                avgAngularRateYB, 0.0);
        assertEquals(avgAngularRateY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(avgAngularRateY1, avgAngularRateY2);

        final AngularSpeed avgAngularRateZ1 = estimator
                .getAvgAngularRateZAsAngularSpeed();
        final AngularSpeed avgAngularRateZ2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAvgAngularRateZAsAngularSpeed(avgAngularRateZ2);

        assertEquals(avgAngularRateZ1.getValue().doubleValue(),
                avgAngularRateZB, 0.0);
        assertEquals(avgAngularRateZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(avgAngularRateZ1, avgAngularRateZ2);

        final BodyKinematics avgKinematics = new BodyKinematics(
                avgFxB, avgFyB, avgFzB, avgAngularRateXB, avgAngularRateYB,
                avgAngularRateZB);

        assertEquals(estimator.getAvgBodyKinematics(), avgKinematics);
        final BodyKinematics avgKinematics2 = new BodyKinematics();
        estimator.getAvgBodyKinematics(avgKinematics2);
        assertEquals(avgKinematics, avgKinematics2);

        final double varFxB = estimator.getVarianceFx();
        final double varFyB = estimator.getVarianceFy();
        final double varFzB = estimator.getVarianceFz();
        final double varOmegaXB = estimator.getVarianceAngularRateX();
        final double varOmegaYB = estimator.getVarianceAngularRateY();
        final double varOmegaZB = estimator.getVarianceAngularRateZ();

        assertEquals(varFxB, varFx, 0.0);
        assertEquals(varFyB, varFy, 0.0);
        assertEquals(varFzB, varFz, 0.0);
        assertEquals(varOmegaXB, varOmegaX, 0.0);
        assertEquals(varOmegaYB, varOmegaY, 0.0);
        assertEquals(varOmegaZB, varOmegaZ, 0.0);

        final double stdFx = estimator.getStandardDeviationFx();
        final double stdFy = estimator.getStandardDeviationFy();
        final double stdFz = estimator.getStandardDeviationFz();
        final double stdOmegaX = estimator.getStandardDeviationAngularRateX();
        final double stdOmegaY = estimator.getStandardDeviationAngularRateY();
        final double stdOmegaZ = estimator.getStandardDeviationAngularRateZ();

        final double avgStdF = (stdFx + stdFy + stdFz) / 3.0;
        final double avgStdOmega = (stdOmegaX + stdOmegaY + stdOmegaZ) / 3.0;

        assertEquals(avgStdF, estimator.getAverageAccelerometerStandardDeviation(), 0.0);
        assertEquals(avgStdOmega, estimator.getAverageGyroscopeStandardDeviation(), 0.0);

        assertEquals(stdFx, Math.sqrt(varFx), 0.0);
        assertEquals(stdFy, Math.sqrt(varFy), 0.0);
        assertEquals(stdFz, Math.sqrt(varFz), 0.0);
        assertEquals(stdOmegaX, Math.sqrt(varOmegaX), 0.0);
        assertEquals(stdOmegaY, Math.sqrt(varOmegaY), 0.0);
        assertEquals(stdOmegaZ, Math.sqrt(varOmegaZ), 0.0);

        final Acceleration stdFx1 = estimator.getStandardDeviationFxAsAcceleration();
        final Acceleration stdFx2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationFxAsAcceleration(stdFx2);
        assertEquals(stdFx1.getValue().doubleValue(), stdFx, 0.0);
        assertEquals(stdFx1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(stdFx1, stdFx2);

        final Acceleration stdFy1 = estimator.getStandardDeviationFyAsAcceleration();
        final Acceleration stdFy2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationFyAsAcceleration(stdFy2);
        assertEquals(stdFy1.getValue().doubleValue(), stdFy, 0.0);
        assertEquals(stdFy1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(stdFy1, stdFy2);

        final Acceleration stdFz1 = estimator.getStandardDeviationFzAsAcceleration();
        final Acceleration stdFz2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationFzAsAcceleration(stdFz2);
        assertEquals(stdFz1.getValue().doubleValue(), stdFz, 0.0);
        assertEquals(stdFz1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(stdFz1, stdFz2);

        final Acceleration avgF1 = estimator
                .getAverageAccelerometerStandardDeviationAsAcceleration();
        final Acceleration avgF2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAverageAccelerometerStandardDeviationAsAcceleration(avgF2);

        assertEquals(avgF1.getValue().doubleValue(), avgStdF, 0.0);
        assertEquals(avgF1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(avgF1, avgF2);

        final AngularSpeed stdOmegaX1 = estimator
                .getStandardDeviationAngularRateXAsAngularSpeed();
        final AngularSpeed stdOmegaX2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateXAsAngularSpeed(stdOmegaX2);
        assertEquals(stdOmegaX1.getValue().doubleValue(), stdOmegaX, 0.0);
        assertEquals(stdOmegaX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(stdOmegaX1, stdOmegaX2);

        final AngularSpeed stdOmegaY1 = estimator
                .getStandardDeviationAngularRateYAsAngularSpeed();
        final AngularSpeed stdOmegaY2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateYAsAngularSpeed(stdOmegaY2);
        assertEquals(stdOmegaY1.getValue().doubleValue(), stdOmegaY, 0.0);
        assertEquals(stdOmegaY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(stdOmegaY1, stdOmegaY2);

        final AngularSpeed stdOmegaZ1 = estimator
                .getStandardDeviationAngularRateZAsAngularSpeed();
        final AngularSpeed stdOmegaZ2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateZAsAngularSpeed(stdOmegaZ2);
        assertEquals(stdOmegaZ1.getValue().doubleValue(), stdOmegaZ, 0.0);
        assertEquals(stdOmegaZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(stdOmegaZ1, stdOmegaZ2);

        final AngularSpeed avgAngularRate1 = estimator
                .getAverageGyroscopeStandardDeviationAsAngularSpeed();
        final AngularSpeed avgAngularRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAverageGyroscopeStandardDeviationAsAngularSpeed(avgAngularRate2);

        assertEquals(avgAngularRate1.getValue().doubleValue(), avgStdOmega, 0.0);
        assertEquals(avgAngularRate1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(avgAngularRate1, avgAngularRate2);

        final BodyKinematics stdKinematics = new BodyKinematics(stdFx, stdFy, stdFz,
                stdOmegaX, stdOmegaY, stdOmegaZ);

        assertEquals(stdKinematics, estimator.getStandardDeviationsAsBodyKinematics());
        final BodyKinematics stdKinematics2 = new BodyKinematics();
        estimator.getStandardDeviationsAsBodyKinematics(stdKinematics2);
        assertEquals(stdKinematics, stdKinematics2);

        final double psdFx = estimator.getPSDFx();
        final double psdFy = estimator.getPSDFy();
        final double psdFz = estimator.getPSDFz();
        final double psdOmegaX = estimator.getPSDAngularRateX();
        final double psdOmegaY = estimator.getPSDAngularRateY();
        final double psdOmegaZ = estimator.getPSDAngularRateZ();

        assertEquals(psdFx, varFx * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdFy, varFy * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdFz, varFz * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdOmegaX, varOmegaX * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdOmegaY, varOmegaY * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdOmegaZ, varOmegaZ * timeInterval, SMALL_ABSOLUTE_ERROR);

        final double rootPsdFx = estimator.getRootPSDFx();
        final double rootPsdFy = estimator.getRootPSDFy();
        final double rootPsdFz = estimator.getRootPSDFz();
        final double rootPsdOmegaX = estimator.getRootPSDAngularRateX();
        final double rootPsdOmegaY = estimator.getRootPSDAngularRateY();
        final double rootPsdOmegaZ = estimator.getRootPSDAngularRateZ();

        assertEquals(rootPsdFx, Math.sqrt(varFx  * timeInterval),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(rootPsdFy,Math.sqrt(varFy * timeInterval),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(rootPsdFz, Math.sqrt(varFz * timeInterval),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(rootPsdOmegaX, Math.sqrt(varOmegaX * timeInterval),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(rootPsdOmegaY, Math.sqrt(varOmegaY * timeInterval),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(rootPsdOmegaZ, Math.sqrt(varOmegaZ * timeInterval),
                SMALL_ABSOLUTE_ERROR);

        assertEquals(rootPsdFx, accelNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdFy, accelNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdFz, accelNoiseRootPSD, ABSOLUTE_ERROR);

        assertEquals(rootPsdOmegaX, gyroNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdOmegaY, gyroNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdOmegaZ, gyroNoiseRootPSD, ABSOLUTE_ERROR);

        final double accelNoisePsd = accelNoiseRootPSD * accelNoiseRootPSD;
        assertEquals(estimator.getAccelerometerNoisePSD(),
                accelNoisePsd, ABSOLUTE_ERROR);
        assertEquals(estimator.getAccelerometerNoiseRootPSD(),
                accelNoiseRootPSD, ABSOLUTE_ERROR);

        final double gyroNoisePsd = gyroNoiseRootPSD * gyroNoiseRootPSD;
        assertEquals(estimator.getGyroNoisePSD(), gyroNoisePsd, ABSOLUTE_ERROR);
        assertEquals(estimator.getGyroNoiseRootPSD(),
                gyroNoiseRootPSD, ABSOLUTE_ERROR);

        // reset
        estimator.reset();

        assertFalse(estimator.isFinished());
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertFalse(estimator.isRunning());
        assertEquals(mReset, 1);
    }

    @Override
    public void onStart(final IMUNoiseEstimator estimator) {
        checkLocked(estimator);
        mStart++;
    }

    @Override
    public void onBodyKinematicsAdded(final IMUNoiseEstimator estimator) {
        if (mBodyKinematicsAdded == 0) {
            checkLocked(estimator);
        }
        mBodyKinematicsAdded++;
    }

    @Override
    public void onFinish(final IMUNoiseEstimator estimator) {
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isFinished());
        mFinish++;
    }

    @Override
    public void onReset(final IMUNoiseEstimator estimator) {
        checkLocked(estimator);
        mReset++;
    }

    private void reset() {
        mStart = 0;
        mBodyKinematicsAdded = 0;
        mFinish = 0;
        mReset = 0;
    }

    private void checkLocked(final IMUNoiseEstimator estimator) {
        assertTrue(estimator.isRunning());
        try {
            estimator.setTotalSamples(0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setTimeInterval(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setTimeInterval(new Time(0.0, TimeUnit.SECOND));
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.addBodyKinematics(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.reset();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
    }

    private Matrix generateBa() {
        return Matrix.newFromArray(new double[]{
                900 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                -1300 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                800 * MICRO_G_TO_METERS_PER_SECOND_SQUARED});
    }

    private Matrix generateBg() {
        return Matrix.newFromArray(new double[]{
                -9 * DEG_TO_RAD / 3600.0,
                13 * DEG_TO_RAD / 3600.0,
                -8 * DEG_TO_RAD / 3600.0});
    }

    private Matrix generateMa() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                -150e-6, -600e-6, 250e-6,
                -250e-6, 100e-6, 450e-6
        }, false);

        return result;
    }

    private Matrix generateMg() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                400e-6, -300e-6, 250e-6,
                0.0, -300e-6, -150e-6,
                0.0, 0.0, -350e-6
        }, false);

        return result;
    }

    private Matrix generateGg() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        final double tmp = DEG_TO_RAD / (3600 * 9.80665);
        result.fromArray(new double[]{
                0.9 * tmp, -1.1 * tmp, -0.6 * tmp,
                -0.5 * tmp, 1.9 * tmp, -1.6 * tmp,
                0.3 * tmp, 1.1 * tmp, -1.3 * tmp
        }, false);

        return result;
    }

    private double getAccelNoiseRootPSD() {
        return 100.0 * MICRO_G_TO_METERS_PER_SECOND_SQUARED;
    }

    private double getGyroNoiseRootPSD() {
        return 0.01 * DEG_TO_RAD / 60.0;
    }
}
