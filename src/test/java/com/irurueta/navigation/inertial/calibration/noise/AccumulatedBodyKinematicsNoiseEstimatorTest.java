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
package com.irurueta.navigation.inertial.calibration.noise;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
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

public class AccumulatedBodyKinematicsNoiseEstimatorTest implements
        AccumulatedBodyKinematicsNoiseEstimatorListener {

    private static final double MIN_ACCELEROMETER_VALUE = -2.0 * 9.81;
    private static final double MAX_ACCELEROMETER_VALUE = 2.0 * 9.81;

    private static final double MIN_GYRO_VALUE = -2.0;
    private static final double MAX_GYRO_VALUE = 2.0;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double ABSOLUTE_ERROR = 1e-5;
    private static final double SMALL_ABSOLUTE_ERROR = 1e-12;
    private static final double LARGE_ABSOLUTE_ERROR = 0.1;

    private static final int N_SAMPLES = 100000;

    private static final int TIMES = 5;

    private int mStart;
    private int mBodyKinematicsAdded;
    private int mReset;

    @Test
    public void testConstructor1() {
        final AccumulatedBodyKinematicsNoiseEstimator estimator =
                new AccumulatedBodyKinematicsNoiseEstimator();

        // check default values
        assertEquals(AccumulatedBodyKinematicsNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedBodyKinematicsNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());
        final Time t2 = new Time(1.0, TimeUnit.NANOSECOND);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getAvgSpecificForceX(), 0.0);
        final Acceleration avgFx1 = estimator.getAvgSpecificForceXAsMeasurement();
        assertEquals(0.0, avgFx1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFx1.getUnit());
        final Acceleration avgFx2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceXAsMeasurement(avgFx2);
        assertEquals(avgFx1, avgFx2);
        assertEquals(0.0, estimator.getAvgSpecificForceY(), 0.0);
        final Acceleration avgFy1 = estimator.getAvgSpecificForceYAsMeasurement();
        assertEquals(0.0, avgFy1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFx1.getUnit());
        final Acceleration avgFy2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceYAsMeasurement(avgFy2);
        assertEquals(avgFy1, avgFy2);
        assertEquals(0.0, estimator.getAvgSpecificForceZ(), 0.0);
        final Acceleration avgFz1 = estimator.getAvgSpecificForceZAsMeasurement();
        assertEquals(0.0, avgFz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFz1.getUnit());
        final Acceleration avgFz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceZAsMeasurement(avgFz2);
        assertEquals(avgFz1, avgFz2);
        final AccelerationTriad avgFTriad1 = estimator.getAvgSpecificForceAsTriad();
        assertEquals(0.0, avgFTriad1.getValueX(), 0.0);
        assertEquals(0.0, avgFTriad1.getValueY(), 0.0);
        assertEquals(0.0, avgFTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFTriad1.getUnit());
        final AccelerationTriad avgFTriad2 = new AccelerationTriad();
        estimator.getAvgSpecificForceAsTriad(avgFTriad2);
        assertEquals(avgFTriad1, avgFTriad2);
        assertEquals(0.0, estimator.getAvgSpecificForceNorm(), 0.0);
        final Acceleration avgFNorm1 = estimator.getAvgSpecificForceNormAsMeasurement();
        assertEquals(0.0, avgFNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFNorm1.getUnit());
        final Acceleration avgFNorm2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceNormAsMeasurement(avgFNorm2);
        assertEquals(avgFNorm1, avgFNorm2);
        assertEquals(0.0, estimator.getAvgAngularRateX(), 0.0);
        final AngularSpeed avgWx1 = estimator.getAvgAngularRateXAsMeasurement();
        assertEquals(0.0, avgWx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWx1.getUnit());
        final AngularSpeed avgWx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateXAsMeasurement(avgWx2);
        assertEquals(avgWx1, avgWx2);
        assertEquals(0.0, estimator.getAvgAngularRateY(), 0.0);
        final AngularSpeed avgWy1 = estimator.getAvgAngularRateYAsMeasurement();
        assertEquals(0.0, avgWy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWy1.getUnit());
        final AngularSpeed avgWy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateYAsMeasurement(avgWy2);
        assertEquals(avgWy1, avgWy2);
        assertEquals(0.0, estimator.getAvgAngularRateZ(), 0.0);
        final AngularSpeed avgWz1 = estimator.getAvgAngularRateZAsMeasurement();
        assertEquals(0.0, avgWz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWz1.getUnit());
        final AngularSpeed avgWz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateZAsMeasurement(avgWz2);
        assertEquals(avgWz1, avgWz2);
        final AngularSpeedTriad avgWTriad1 = estimator.getAvgAngularRateTriad();
        assertEquals(0.0, avgWTriad1.getValueX(), 0.0);
        assertEquals(0.0, avgWTriad1.getValueY(), 0.0);
        assertEquals(0.0, avgWTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWTriad1.getUnit());
        final AngularSpeedTriad avgWTriad2 = new AngularSpeedTriad();
        estimator.getAvgAngularRateTriad(avgWTriad2);
        assertEquals(avgWTriad1, avgWTriad2);
        assertEquals(0.0, estimator.getAvgAngularRateNorm(), 0.0);
        final AngularSpeed avgWNorm1 = estimator.getAvgAngularRateNormAsMeasurement();
        assertEquals(0.0, avgWNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWNorm1.getUnit());
        final AngularSpeed avgWNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateNormAsMeasurement(avgWNorm2);
        assertEquals(avgWNorm1, avgWNorm2);
        final BodyKinematics avgKinematics1 = estimator.getAvgBodyKinematics();
        assertEquals(0.0, avgKinematics1.getFx(), 0.0);
        assertEquals(0.0, avgKinematics1.getFy(), 0.0);
        assertEquals(0.0, avgKinematics1.getFz(), 0.0);
        assertEquals(0.0, avgKinematics1.getAngularRateX(), 0.0);
        assertEquals(0.0, avgKinematics1.getAngularRateY(), 0.0);
        assertEquals(0.0, avgKinematics1.getAngularRateZ(), 0.0);
        final BodyKinematics avgKinematics2 = new BodyKinematics();
        estimator.getAvgBodyKinematics(avgKinematics2);
        assertEquals(avgKinematics1, avgKinematics2);
        assertEquals(0.0, estimator.getVarianceSpecificForceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceSpecificForceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceSpecificForceZ(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationSpecificForceX(), 0.0);
        final Acceleration stdFx1 = estimator.getStandardDeviationSpecificForceXAsMeasurement();
        assertEquals(0.0, stdFx1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFx1.getUnit());
        final Acceleration stdFx2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceXAsMeasurement(stdFx2);
        assertEquals(stdFx1, stdFx2);
        assertEquals(0.0, estimator.getStandardDeviationSpecificForceY(), 0.0);
        final Acceleration stdFy1 = estimator.getStandardDeviationSpecificForceYAsMeasurement();
        assertEquals(0.0, stdFy1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFy1.getUnit());
        final Acceleration stdFy2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceYAsMeasurement(stdFy2);
        assertEquals(stdFy1, stdFy2);
        assertEquals(0.0, estimator.getStandardDeviationSpecificForceZ(), 0.0);
        final Acceleration stdFz1 = estimator.getStandardDeviationSpecificForceZAsMeasurement();
        assertEquals(0.0, stdFz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFz1.getUnit());
        final Acceleration stdFz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceZAsMeasurement(stdFz2);
        assertEquals(stdFz1, stdFz2);
        final AccelerationTriad stdFTriad1 = estimator.getStandardDeviationSpecificForceTriad();
        assertEquals(0.0, stdFTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdFTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdFTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFTriad1.getUnit());
        final AccelerationTriad stdFTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationSpecificForceTriad(stdFTriad2);
        assertEquals(stdFTriad1, stdFTriad2);
        assertEquals(0.0, estimator.getStandardDeviationSpecificForceNorm(), 0.0);
        final Acceleration stdFNorm1 = estimator.getStandardDeviationSpecificForceNormAsMeasurement();
        assertEquals(0.0, stdFNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFNorm1.getUnit());
        final Acceleration stdFNorm2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceNormAsMeasurement(stdFNorm2);
        assertEquals(stdFNorm1, stdFNorm2);
        assertEquals(0.0, estimator.getAverageStandardDeviationSpecificForce(), 0.0);
        final Acceleration avgStdF1 = estimator.getAverageStandardDeviationSpecificForceAsMeasurement();
        assertEquals(0.0, avgStdF1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgStdF1.getUnit());
        final Acceleration avgStdF2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAverageStandardDeviationSpecificForceAsMeasurement(avgStdF2);
        assertEquals(avgStdF1, avgStdF2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        final AngularSpeed stdWx1 = estimator.getStandardDeviationAngularRateXAsMeasurement();
        assertEquals(0.0, stdWx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWx1.getUnit());
        final AngularSpeed stdWx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateXAsMeasurement(stdWx2);
        assertEquals(stdWx1, stdWx2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        final AngularSpeed stdWy1 = estimator.getStandardDeviationAngularRateYAsMeasurement();
        assertEquals(0.0, stdWy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWx1.getUnit());
        final AngularSpeed stdWy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateYAsMeasurement(stdWy2);
        assertEquals(stdWy1, stdWy2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        final AngularSpeed stdWz1 = estimator.getStandardDeviationAngularRateZAsMeasurement();
        assertEquals(0.0, stdWz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWz1.getUnit());
        final AngularSpeed stdWz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateZAsMeasurement(stdWz2);
        assertEquals(stdWz1, stdWz2);
        final AngularSpeedTriad stdWTriad1 = estimator.getStandardDeviationAngularSpeedTriad();
        assertEquals(0.0, stdWTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdWTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdWTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWTriad1.getUnit());
        final AngularSpeedTriad stdWTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularSpeedTriad(stdWTriad2);
        assertEquals(stdWTriad1, stdWTriad2);
        assertEquals(0.0, estimator.getStandardDeviationAngularSpeedNorm(), 0.0);
        final AngularSpeed stdWNorm1 = estimator.getStandardDeviationAngularSpeedNormAsMeasurement();
        assertEquals(0.0, stdWNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWNorm1.getUnit());
        final AngularSpeed stdWNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularSpeedNormAsMeasurement(stdWNorm2);
        assertEquals(stdWNorm1, stdWNorm2);
        assertEquals(0.0, estimator.getAverageStandardDeviationAngularSpeed(),
                0.0);
        final AngularSpeed avgStdW1 = estimator
                .getAverageStandardDeviationAngularSpeedAsMeasurement();
        assertEquals(0.0, avgStdW1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgStdW1.getUnit());
        final AngularSpeed avgStdW2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAverageStandardDeviationAngularSpeedAsMeasurement(avgStdW2);
        assertEquals(avgStdW1, avgStdW2);
        final BodyKinematics stdKinematics1 = estimator.getStandardDeviationAsBodyKinematics();
        assertEquals(0.0, stdKinematics1.getFx(), 0.0);
        assertEquals(0.0, stdKinematics1.getFy(), 0.0);
        assertEquals(0.0, stdKinematics1.getFz(), 0.0);
        assertEquals(0.0, stdKinematics1.getAngularRateX(), 0.0);
        assertEquals(0.0, stdKinematics1.getAngularRateY(), 0.0);
        assertEquals(0.0, stdKinematics1.getAngularRateZ(), 0.0);
        final BodyKinematics stdKinematics2 = new BodyKinematics();
        estimator.getStandardDeviationAsBodyKinematics(stdKinematics2);
        assertEquals(stdKinematics1, stdKinematics2);
        assertEquals(0.0, estimator.getSpecificForcePsdX(), 0.0);
        assertEquals(0.0, estimator.getSpecificForcePsdY(), 0.0);
        assertEquals(0.0, estimator.getSpecificForcePsdZ(), 0.0);
        assertEquals(0.0, estimator.getAngularRatePsdX(), 0.0);
        assertEquals(0.0, estimator.getAngularRatePsdY(), 0.0);
        assertEquals(0.0, estimator.getAngularRatePsdZ(), 0.0);
        assertEquals(0.0, estimator.getSpecificForceRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getSpecificForceRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getSpecificForceRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAngularRateRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getAngularRateRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getAngularRateRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgSpecificForceNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getSpecificForceNoiseRootPsdNorm(), 0.0);
        assertEquals(0.0, estimator.getAvgAngularRateNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getAngularRateNoiseRootPsdNorm(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Test
    public void testConstructor2() {
        final AccumulatedBodyKinematicsNoiseEstimator estimator =
                new AccumulatedBodyKinematicsNoiseEstimator(this);

        // check default values
        assertEquals(AccumulatedBodyKinematicsNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedBodyKinematicsNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());
        final Time t2 = new Time(1.0, TimeUnit.NANOSECOND);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertEquals(0.0, estimator.getAvgSpecificForceX(), 0.0);
        final Acceleration avgFx1 = estimator.getAvgSpecificForceXAsMeasurement();
        assertEquals(0.0, avgFx1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFx1.getUnit());
        final Acceleration avgFx2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceXAsMeasurement(avgFx2);
        assertEquals(avgFx1, avgFx2);
        assertEquals(0.0, estimator.getAvgSpecificForceY(), 0.0);
        final Acceleration avgFy1 = estimator.getAvgSpecificForceYAsMeasurement();
        assertEquals(0.0, avgFy1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFx1.getUnit());
        final Acceleration avgFy2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceYAsMeasurement(avgFy2);
        assertEquals(avgFy1, avgFy2);
        assertEquals(0.0, estimator.getAvgSpecificForceZ(), 0.0);
        final Acceleration avgFz1 = estimator.getAvgSpecificForceZAsMeasurement();
        assertEquals(0.0, avgFz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFz1.getUnit());
        final Acceleration avgFz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceZAsMeasurement(avgFz2);
        assertEquals(avgFz1, avgFz2);
        final AccelerationTriad avgFTriad1 = estimator.getAvgSpecificForceAsTriad();
        assertEquals(0.0, avgFTriad1.getValueX(), 0.0);
        assertEquals(0.0, avgFTriad1.getValueY(), 0.0);
        assertEquals(0.0, avgFTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFTriad1.getUnit());
        final AccelerationTriad avgFTriad2 = new AccelerationTriad();
        estimator.getAvgSpecificForceAsTriad(avgFTriad2);
        assertEquals(avgFTriad1, avgFTriad2);
        assertEquals(0.0, estimator.getAvgSpecificForceNorm(), 0.0);
        final Acceleration avgFNorm1 = estimator.getAvgSpecificForceNormAsMeasurement();
        assertEquals(0.0, avgFNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFNorm1.getUnit());
        final Acceleration avgFNorm2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceNormAsMeasurement(avgFNorm2);
        assertEquals(avgFNorm1, avgFNorm2);
        assertEquals(0.0, estimator.getAvgAngularRateX(), 0.0);
        final AngularSpeed avgWx1 = estimator.getAvgAngularRateXAsMeasurement();
        assertEquals(0.0, avgWx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWx1.getUnit());
        final AngularSpeed avgWx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateXAsMeasurement(avgWx2);
        assertEquals(avgWx1, avgWx2);
        assertEquals(0.0, estimator.getAvgAngularRateY(), 0.0);
        final AngularSpeed avgWy1 = estimator.getAvgAngularRateYAsMeasurement();
        assertEquals(0.0, avgWy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWy1.getUnit());
        final AngularSpeed avgWy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateYAsMeasurement(avgWy2);
        assertEquals(avgWy1, avgWy2);
        assertEquals(0.0, estimator.getAvgAngularRateZ(), 0.0);
        final AngularSpeed avgWz1 = estimator.getAvgAngularRateZAsMeasurement();
        assertEquals(0.0, avgWz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWz1.getUnit());
        final AngularSpeed avgWz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateZAsMeasurement(avgWz2);
        assertEquals(avgWz1, avgWz2);
        final AngularSpeedTriad avgWTriad1 = estimator.getAvgAngularRateTriad();
        assertEquals(0.0, avgWTriad1.getValueX(), 0.0);
        assertEquals(0.0, avgWTriad1.getValueY(), 0.0);
        assertEquals(0.0, avgWTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWTriad1.getUnit());
        final AngularSpeedTriad avgWTriad2 = new AngularSpeedTriad();
        estimator.getAvgAngularRateTriad(avgWTriad2);
        assertEquals(avgWTriad1, avgWTriad2);
        assertEquals(0.0, estimator.getAvgAngularRateNorm(), 0.0);
        final AngularSpeed avgWNorm1 = estimator.getAvgAngularRateNormAsMeasurement();
        assertEquals(0.0, avgWNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWNorm1.getUnit());
        final AngularSpeed avgWNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateNormAsMeasurement(avgWNorm2);
        assertEquals(avgWNorm1, avgWNorm2);
        final BodyKinematics avgKinematics1 = estimator.getAvgBodyKinematics();
        assertEquals(0.0, avgKinematics1.getFx(), 0.0);
        assertEquals(0.0, avgKinematics1.getFy(), 0.0);
        assertEquals(0.0, avgKinematics1.getFz(), 0.0);
        assertEquals(0.0, avgKinematics1.getAngularRateX(), 0.0);
        assertEquals(0.0, avgKinematics1.getAngularRateY(), 0.0);
        assertEquals(0.0, avgKinematics1.getAngularRateZ(), 0.0);
        final BodyKinematics avgKinematics2 = new BodyKinematics();
        estimator.getAvgBodyKinematics(avgKinematics2);
        assertEquals(avgKinematics1, avgKinematics2);
        assertEquals(0.0, estimator.getVarianceSpecificForceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceSpecificForceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceSpecificForceZ(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateX(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateY(), 0.0);
        assertEquals(0.0, estimator.getVarianceAngularRateZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationSpecificForceX(), 0.0);
        final Acceleration stdFx1 = estimator.getStandardDeviationSpecificForceXAsMeasurement();
        assertEquals(0.0, stdFx1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFx1.getUnit());
        final Acceleration stdFx2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceXAsMeasurement(stdFx2);
        assertEquals(stdFx1, stdFx2);
        assertEquals(0.0, estimator.getStandardDeviationSpecificForceY(), 0.0);
        final Acceleration stdFy1 = estimator.getStandardDeviationSpecificForceYAsMeasurement();
        assertEquals(0.0, stdFy1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFy1.getUnit());
        final Acceleration stdFy2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceYAsMeasurement(stdFy2);
        assertEquals(stdFy1, stdFy2);
        assertEquals(0.0, estimator.getStandardDeviationSpecificForceZ(), 0.0);
        final Acceleration stdFz1 = estimator.getStandardDeviationSpecificForceZAsMeasurement();
        assertEquals(0.0, stdFz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFz1.getUnit());
        final Acceleration stdFz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceZAsMeasurement(stdFz2);
        assertEquals(stdFz1, stdFz2);
        final AccelerationTriad stdFTriad1 = estimator.getStandardDeviationSpecificForceTriad();
        assertEquals(0.0, stdFTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdFTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdFTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFTriad1.getUnit());
        final AccelerationTriad stdFTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationSpecificForceTriad(stdFTriad2);
        assertEquals(stdFTriad1, stdFTriad2);
        assertEquals(0.0, estimator.getStandardDeviationSpecificForceNorm(), 0.0);
        final Acceleration stdFNorm1 = estimator.getStandardDeviationSpecificForceNormAsMeasurement();
        assertEquals(0.0, stdFNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, stdFNorm1.getUnit());
        final Acceleration stdFNorm2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceNormAsMeasurement(stdFNorm2);
        assertEquals(stdFNorm1, stdFNorm2);
        assertEquals(0.0, estimator.getAverageStandardDeviationSpecificForce(), 0.0);
        final Acceleration avgStdF1 = estimator.getAverageStandardDeviationSpecificForceAsMeasurement();
        assertEquals(0.0, avgStdF1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgStdF1.getUnit());
        final Acceleration avgStdF2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAverageStandardDeviationSpecificForceAsMeasurement(avgStdF2);
        assertEquals(avgStdF1, avgStdF2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateX(), 0.0);
        final AngularSpeed stdWx1 = estimator.getStandardDeviationAngularRateXAsMeasurement();
        assertEquals(0.0, stdWx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWx1.getUnit());
        final AngularSpeed stdWx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateXAsMeasurement(stdWx2);
        assertEquals(stdWx1, stdWx2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateY(), 0.0);
        final AngularSpeed stdWy1 = estimator.getStandardDeviationAngularRateYAsMeasurement();
        assertEquals(0.0, stdWy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWx1.getUnit());
        final AngularSpeed stdWy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateYAsMeasurement(stdWy2);
        assertEquals(stdWy1, stdWy2);
        assertEquals(0.0, estimator.getStandardDeviationAngularRateZ(), 0.0);
        final AngularSpeed stdWz1 = estimator.getStandardDeviationAngularRateZAsMeasurement();
        assertEquals(0.0, stdWz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWz1.getUnit());
        final AngularSpeed stdWz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateZAsMeasurement(stdWz2);
        assertEquals(stdWz1, stdWz2);
        final AngularSpeedTriad stdWTriad1 = estimator.getStandardDeviationAngularSpeedTriad();
        assertEquals(0.0, stdWTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdWTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdWTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWTriad1.getUnit());
        final AngularSpeedTriad stdWTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularSpeedTriad(stdWTriad2);
        assertEquals(stdWTriad1, stdWTriad2);
        assertEquals(0.0, estimator.getStandardDeviationAngularSpeedNorm(), 0.0);
        final AngularSpeed stdWNorm1 = estimator.getStandardDeviationAngularSpeedNormAsMeasurement();
        assertEquals(0.0, stdWNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWNorm1.getUnit());
        final AngularSpeed stdWNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularSpeedNormAsMeasurement(stdWNorm2);
        assertEquals(stdWNorm1, stdWNorm2);
        assertEquals(0.0, estimator.getAverageStandardDeviationAngularSpeed(),
                0.0);
        final AngularSpeed avgStdW1 = estimator
                .getAverageStandardDeviationAngularSpeedAsMeasurement();
        assertEquals(0.0, avgStdW1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgStdW1.getUnit());
        final AngularSpeed avgStdW2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAverageStandardDeviationAngularSpeedAsMeasurement(avgStdW2);
        assertEquals(avgStdW1, avgStdW2);
        final BodyKinematics stdKinematics1 = estimator.getStandardDeviationAsBodyKinematics();
        assertEquals(0.0, stdKinematics1.getFx(), 0.0);
        assertEquals(0.0, stdKinematics1.getFy(), 0.0);
        assertEquals(0.0, stdKinematics1.getFz(), 0.0);
        assertEquals(0.0, stdKinematics1.getAngularRateX(), 0.0);
        assertEquals(0.0, stdKinematics1.getAngularRateY(), 0.0);
        assertEquals(0.0, stdKinematics1.getAngularRateZ(), 0.0);
        final BodyKinematics stdKinematics2 = new BodyKinematics();
        estimator.getStandardDeviationAsBodyKinematics(stdKinematics2);
        assertEquals(stdKinematics1, stdKinematics2);
        assertEquals(0.0, estimator.getSpecificForcePsdX(), 0.0);
        assertEquals(0.0, estimator.getSpecificForcePsdY(), 0.0);
        assertEquals(0.0, estimator.getSpecificForcePsdZ(), 0.0);
        assertEquals(0.0, estimator.getAngularRatePsdX(), 0.0);
        assertEquals(0.0, estimator.getAngularRatePsdY(), 0.0);
        assertEquals(0.0, estimator.getAngularRatePsdZ(), 0.0);
        assertEquals(0.0, estimator.getSpecificForceRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getSpecificForceRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getSpecificForceRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAngularRateRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getAngularRateRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getAngularRateRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgSpecificForceNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getSpecificForceNoiseRootPsdNorm(), 0.0);
        assertEquals(0.0, estimator.getAvgAngularRateNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getAngularRateNoiseRootPsdNorm(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Test
    public void testGetSetTimeInterval() throws LockedException {
        final AccumulatedBodyKinematicsNoiseEstimator estimator =
                new AccumulatedBodyKinematicsNoiseEstimator();

        // check default value
        assertEquals(
                AccumulatedBodyKinematicsNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        // set new value
        estimator.setTimeInterval(1.0);

        // check
        assertEquals(1.0, estimator.getTimeInterval(), 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setTimeInterval(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetTimeIntervalAsTime() throws LockedException {
        final AccumulatedBodyKinematicsNoiseEstimator estimator =
                new AccumulatedBodyKinematicsNoiseEstimator();

        // check default value
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(
                AccumulatedBodyKinematicsNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());

        // set new value
        final Time time2 = new Time(500, TimeUnit.MILLISECOND);
        estimator.setTimeInterval(time2);

        // check
        final Time time3 = estimator.getTimeIntervalAsTime();
        final Time time4 = new Time(0.0, TimeUnit.SECOND);
        estimator.getTimeIntervalAsTime(time4);

        assertTrue(time2.equals(time3, ABSOLUTE_ERROR));
        assertTrue(time2.equals(time4, ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final AccumulatedBodyKinematicsNoiseEstimator estimator =
                new AccumulatedBodyKinematicsNoiseEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testAddBodyKinematicsAndReset1()
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

        final AccumulatedBodyKinematicsNoiseEstimator estimator =
                new AccumulatedBodyKinematicsNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mBodyKinematicsAdded, 0);
        assertEquals(mReset, 0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertFalse(estimator.isRunning());

        final BodyKinematics kinematics = new BodyKinematics();
        final double timeInterval = estimator.getTimeInterval();
        BodyKinematics lastKinematics = new BodyKinematics();

        double avgFx = 0.0;
        double avgFy = 0.0;
        double avgFz = 0.0;
        double avgWx = 0.0;
        double avgWy = 0.0;
        double avgWz = 0.0;
        double varFx = 0.0;
        double varFy = 0.0;
        double varFz = 0.0;
        double varWx = 0.0;
        double varWy = 0.0;
        double varWz = 0.0;
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastBodyKinematics(lastKinematics)) {
                assertEquals(estimator.getLastBodyKinematics(), lastKinematics);
                assertEquals(lastKinematics, kinematics);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                    errors, random, kinematics);

            final double fxi = kinematics.getFx();
            final double fyi = kinematics.getFy();
            final double fzi = kinematics.getFz();
            final double wxi = kinematics.getAngularRateX();
            final double wyi = kinematics.getAngularRateY();
            final double wzi = kinematics.getAngularRateZ();

            estimator.addBodyKinematics(fxi, fyi, fzi, wxi, wyi, wzi);

            assertTrue(estimator.getLastBodyKinematics(lastKinematics));
            assertEquals(lastKinematics, kinematics);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avgFx = avgFx * (double) i / (double) j + fxi / j;
            avgFy = avgFy * (double) i / (double) j + fyi / j;
            avgFz = avgFz * (double) i / (double) j + fzi / j;

            avgWx = avgWx * (double) i / (double) j + wxi / j;
            avgWy = avgWy * (double) i / (double) j + wyi / j;
            avgWz = avgWz * (double) i / (double) j + wzi / j;

            double diff = fxi - avgFx;
            double diff2 = diff * diff;
            varFx = varFx * (double) i / (double) j + diff2 / j;

            diff = fyi - avgFy;
            diff2 = diff * diff;
            varFy = varFy * (double) i / (double) j + diff2 / j;

            diff = fzi - avgFz;
            diff2 = diff * diff;
            varFz = varFz * (double) i / (double) j + diff2 / j;

            diff = wxi - avgWx;
            diff2 = diff * diff;
            varWx = varWx * (double) i / (double) j + diff2 / j;

            diff = wyi - avgWy;
            diff2 = diff * diff;
            varWy = varWy * (double) i / (double) j + diff2 / j;

            diff = wzi - avgWz;
            diff2 = diff * diff;
            varWz = varWz * (double) i / (double) j + diff2 / j;
        }

        assertEquals(estimator.getNumberOfProcessedSamples(), N_SAMPLES);
        assertFalse(estimator.isRunning());
        assertEquals(mStart, 1);
        assertEquals(mBodyKinematicsAdded, N_SAMPLES);
        assertEquals(mReset, 0);

        final double avgFxB = estimator.getAvgSpecificForceX();
        final double avgFyB = estimator.getAvgSpecificForceY();
        final double avgFzB = estimator.getAvgSpecificForceZ();

        final double avgWxB = estimator.getAvgAngularRateX();
        final double avgWyB = estimator.getAvgAngularRateY();
        final double avgWzB = estimator.getAvgAngularRateZ();

        assertEquals(avgFx, avgFxB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFy, avgFyB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFz, avgFzB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWx, avgWxB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWy, avgWyB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWz, avgWzB, SMALL_ABSOLUTE_ERROR);

        assertEquals(avgFxB, fx, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgFyB, fy, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgFzB, fz, LARGE_ABSOLUTE_ERROR);

        assertEquals(avgWxB, omegaX, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgWyB, omegaY, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgWzB, omegaZ, LARGE_ABSOLUTE_ERROR);

        final Acceleration avgFx1 = estimator.getAvgSpecificForceXAsMeasurement();
        final Acceleration avgFx2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceXAsMeasurement(avgFx2);

        assertEquals(avgFx, avgFx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFx1.getUnit());
        assertEquals(avgFx1, avgFx2);

        final Acceleration avgFy1 = estimator.getAvgSpecificForceYAsMeasurement();
        final Acceleration avgFy2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceYAsMeasurement(avgFy2);

        assertEquals(avgFy, avgFy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFy1.getUnit());
        assertEquals(avgFy1, avgFy2);

        final Acceleration avgFz1 = estimator.getAvgSpecificForceZAsMeasurement();
        final Acceleration avgFz2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceZAsMeasurement(avgFz2);

        assertEquals(avgFz, avgFz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFz1.getUnit());
        assertEquals(avgFz1, avgFz2);

        final AccelerationTriad avgFTriad1 = estimator.getAvgSpecificForceAsTriad();
        assertEquals(avgFx, avgFTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFy, avgFTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFz, avgFTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFTriad1.getUnit());
        final AccelerationTriad avgFTriad2 = new AccelerationTriad();
        estimator.getAvgSpecificForceAsTriad(avgFTriad2);
        assertEquals(avgFTriad1, avgFTriad2);
        assertEquals(avgFTriad1.getNorm(), estimator.getAvgSpecificForceNorm(),
                0.0);
        final Acceleration avgFNorm1 = estimator.getAvgSpecificForceNormAsMeasurement();
        assertEquals(estimator.getAvgSpecificForceNorm(),
                avgFNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFNorm1.getUnit());
        final Acceleration avgFNorm2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceNormAsMeasurement(avgFNorm2);
        assertEquals(avgFNorm1, avgFNorm2);

        final AngularSpeed avgWx1 = estimator.getAvgAngularRateXAsMeasurement();
        assertEquals(avgWx, avgWx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWx1.getUnit());
        final AngularSpeed avgWx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateXAsMeasurement(avgWx2);
        assertEquals(avgWx1, avgWx2);

        final AngularSpeed avgWy1 = estimator.getAvgAngularRateYAsMeasurement();
        assertEquals(avgWy, avgWy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWy1.getUnit());
        final AngularSpeed avgWy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateYAsMeasurement(avgWy2);
        assertEquals(avgWy1, avgWy2);

        final AngularSpeed avgWz1 = estimator.getAvgAngularRateZAsMeasurement();
        assertEquals(avgWz, avgWz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWz1.getUnit());
        final AngularSpeed avgWz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateZAsMeasurement(avgWz2);
        assertEquals(avgWz1, avgWz2);

        final AngularSpeedTriad avgWTriad1 = estimator.getAvgAngularRateTriad();
        assertEquals(avgWx, avgWTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWy, avgWTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWz, avgWTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWTriad1.getUnit());
        final AngularSpeedTriad avgWTriad2 = new AngularSpeedTriad();
        estimator.getAvgAngularRateTriad(avgWTriad2);
        assertEquals(avgWTriad1, avgWTriad2);

        assertEquals(avgWTriad1.getNorm(), estimator.getAvgAngularRateNorm(),
                0.0);
        final AngularSpeed avgWNorm1 = estimator.getAvgAngularRateNormAsMeasurement();
        assertEquals(estimator.getAvgAngularRateNorm(),
                avgWNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWNorm1.getUnit());
        final AngularSpeed avgWNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAvgAngularRateNormAsMeasurement(avgWNorm2);
        assertEquals(avgWNorm1, avgWNorm2);

        final BodyKinematics avgKinematics1 = estimator.getAvgBodyKinematics();
        assertEquals(avgFx, avgKinematics1.getFx(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFy, avgKinematics1.getFy(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFz, avgKinematics1.getFz(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWx, avgKinematics1.getAngularRateX(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWy, avgKinematics1.getAngularRateY(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWz, avgKinematics1.getAngularRateZ(),
                SMALL_ABSOLUTE_ERROR);
        final BodyKinematics avgKinematics2 = new BodyKinematics();
        estimator.getAvgBodyKinematics(avgKinematics2);
        assertEquals(avgKinematics1, avgKinematics2);

        assertEquals(varFx, estimator.getVarianceSpecificForceX(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(varFy, estimator.getVarianceSpecificForceY(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(varFz, estimator.getVarianceSpecificForceZ(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(varWx, estimator.getVarianceAngularRateX(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(varWy, estimator.getVarianceAngularRateY(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(varWz, estimator.getVarianceAngularRateZ(),
                SMALL_ABSOLUTE_ERROR);

        final double stdFx = Math.sqrt(varFx);
        final double stdFy = Math.sqrt(varFy);
        final double stdFz = Math.sqrt(varFz);
        final double stdWx = Math.sqrt(varWx);
        final double stdWy = Math.sqrt(varWy);
        final double stdWz = Math.sqrt(varWz);

        assertEquals(stdFx, estimator.getStandardDeviationSpecificForceX(),
                SMALL_ABSOLUTE_ERROR);
        final Acceleration stdFx1 = estimator.getStandardDeviationSpecificForceXAsMeasurement();
        assertEquals(stdFx, stdFx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                stdFx1.getUnit());
        final Acceleration stdFx2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceXAsMeasurement(stdFx2);
        assertEquals(stdFx1, stdFx2);

        assertEquals(stdFy, estimator.getStandardDeviationSpecificForceY(),
                SMALL_ABSOLUTE_ERROR);
        final Acceleration stdFy1 = estimator.getStandardDeviationSpecificForceYAsMeasurement();
        assertEquals(stdFy, stdFy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                stdFy1.getUnit());
        final Acceleration stdFy2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceYAsMeasurement(stdFy2);
        assertEquals(stdFy1, stdFy2);

        assertEquals(stdFz, estimator.getStandardDeviationSpecificForceZ(),
                SMALL_ABSOLUTE_ERROR);
        final Acceleration stdFz1 = estimator.getStandardDeviationSpecificForceZAsMeasurement();
        assertEquals(stdFz, stdFz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                stdFz1.getUnit());
        final Acceleration stdFz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceZAsMeasurement(stdFz2);
        assertEquals(stdFz1, stdFz2);

        final AccelerationTriad stdFTriad1 = estimator.getStandardDeviationSpecificForceTriad();
        assertEquals(stdFx, stdFTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFy, stdFTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFz, stdFTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                stdFTriad1.getUnit());

        final AccelerationTriad stdFTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationSpecificForceTriad(stdFTriad2);
        assertEquals(stdFTriad1, stdFTriad2);

        assertEquals(stdFTriad1.getNorm(),
                estimator.getStandardDeviationSpecificForceNorm(),
                SMALL_ABSOLUTE_ERROR);
        final Acceleration stdFNorm1 = estimator.getStandardDeviationSpecificForceNormAsMeasurement();
        assertEquals(stdFTriad1.getNorm(), stdFNorm1.getValue().doubleValue(),
                0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                stdFNorm1.getUnit());
        final Acceleration stdFNorm2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceNormAsMeasurement(stdFNorm2);
        assertEquals(stdFNorm1, stdFNorm2);

        final double avgStdF = (stdFx + stdFy + stdFz) / 3.0;
        assertEquals(avgStdF, estimator.getAverageStandardDeviationSpecificForce(),
                SMALL_ABSOLUTE_ERROR);
        final Acceleration avgStdF1 = estimator.getAverageStandardDeviationSpecificForceAsMeasurement();
        assertEquals(avgStdF, avgStdF1.getValue().doubleValue(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgStdF1.getUnit());
        final Acceleration avgStdF2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAverageStandardDeviationSpecificForceAsMeasurement(avgStdF2);
        assertEquals(avgStdF1, avgStdF2);

        assertEquals(stdWx, estimator.getStandardDeviationAngularRateX(),
                SMALL_ABSOLUTE_ERROR);
        final AngularSpeed stdWx1 = estimator.getStandardDeviationAngularRateXAsMeasurement();
        assertEquals(stdWx, stdWx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWx1.getUnit());
        final AngularSpeed stdWx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateXAsMeasurement(stdWx2);
        assertEquals(stdWx1, stdWx2);

        assertEquals(stdWy, estimator.getStandardDeviationAngularRateY(),
                SMALL_ABSOLUTE_ERROR);
        final AngularSpeed stdWy1 = estimator.getStandardDeviationAngularRateYAsMeasurement();
        assertEquals(stdWy, stdWy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWy1.getUnit());
        final AngularSpeed stdWy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateYAsMeasurement(stdWy2);
        assertEquals(stdWy1, stdWy2);

        assertEquals(stdWz, estimator.getStandardDeviationAngularRateZ(),
                SMALL_ABSOLUTE_ERROR);
        final AngularSpeed stdWz1 = estimator.getStandardDeviationAngularRateZAsMeasurement();
        assertEquals(stdWz, stdWz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWz1.getUnit());
        final AngularSpeed stdWz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateZAsMeasurement(stdWz2);
        assertEquals(stdWz1, stdWz2);

        final AngularSpeedTriad stdWTriad1 = estimator.getStandardDeviationAngularSpeedTriad();
        assertEquals(stdWx, stdWTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWy, stdWTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWz, stdWTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWTriad1.getUnit());
        final AngularSpeedTriad stdWTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularSpeedTriad(stdWTriad2);
        assertEquals(stdWTriad1, stdWTriad2);

        assertEquals(stdWTriad1.getNorm(), estimator.getStandardDeviationAngularSpeedNorm(),
                0.0);
        final AngularSpeed stdWNorm1 = estimator.getStandardDeviationAngularSpeedNormAsMeasurement();
        assertEquals(stdWTriad1.getNorm(), stdWNorm1.getValue().doubleValue(),
                0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWNorm1.getUnit());
        final AngularSpeed stdWNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularSpeedNormAsMeasurement(stdWNorm2);
        assertEquals(stdWNorm1, stdWNorm2);

        final double avgStdW = (stdWx + stdWy + stdWz) / 3.0;
        assertEquals(avgStdW, estimator.getAverageStandardDeviationAngularSpeed(),
                SMALL_ABSOLUTE_ERROR);
        final AngularSpeed avgStdW1 = estimator.getAverageStandardDeviationAngularSpeedAsMeasurement();
        assertEquals(avgStdW, avgStdW1.getValue().doubleValue(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgStdW1.getUnit());
        final AngularSpeed avgStdW2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAverageStandardDeviationAngularSpeedAsMeasurement(avgStdW2);
        assertEquals(avgStdW1, avgStdW2);

        final BodyKinematics stdKinematics1 = estimator.getStandardDeviationAsBodyKinematics();
        assertEquals(stdFx, stdKinematics1.getFx(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFy, stdKinematics1.getFy(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFz, stdKinematics1.getFz(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWx, stdKinematics1.getAngularRateX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWy, stdKinematics1.getAngularRateY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWz, stdKinematics1.getAngularRateZ(), SMALL_ABSOLUTE_ERROR);
        final BodyKinematics stdKinematics2 = new BodyKinematics();
        estimator.getStandardDeviationAsBodyKinematics(stdKinematics2);
        assertEquals(stdKinematics1, stdKinematics2);

        final double psdFx = estimator.getSpecificForcePsdX();
        final double psdFy = estimator.getSpecificForcePsdY();
        final double psdFz = estimator.getSpecificForcePsdZ();
        final double psdWx = estimator.getAngularRatePsdX();
        final double psdWy = estimator.getAngularRatePsdY();
        final double psdWz = estimator.getAngularRatePsdZ();

        assertEquals(psdFx, varFx * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdFy, varFy * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdFz, varFz * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdWx, varWx * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdWy, varWy * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdWz, varWz * timeInterval, SMALL_ABSOLUTE_ERROR);

        final double rootPsdFx = Math.sqrt(psdFx);
        final double rootPsdFy = Math.sqrt(psdFy);
        final double rootPsdFz = Math.sqrt(psdFz);
        final double rootPsdWx = Math.sqrt(psdWx);
        final double rootPsdWy = Math.sqrt(psdWy);
        final double rootPsdWz = Math.sqrt(psdWz);

        assertEquals(rootPsdFx, estimator.getSpecificForceRootPsdX(),
                0.0);
        assertEquals(rootPsdFy, estimator.getSpecificForceRootPsdY(),
                0.0);
        assertEquals(rootPsdFz, estimator.getSpecificForceRootPsdZ(),
                0.0);
        assertEquals(rootPsdWx, estimator.getAngularRateRootPsdX(),
                0.0);
        assertEquals(rootPsdWy, estimator.getAngularRateRootPsdY(),
                0.0);
        assertEquals(rootPsdWz, estimator.getAngularRateRootPsdZ(),
                0.0);

        final double avgPsdF = (psdFx + psdFy + psdFz) / 3.0;
        final double avgPsdW = (psdWx + psdWy + psdWz) / 3.0;
        final double normRootPsdF = Math.sqrt(
                rootPsdFx * rootPsdFx + rootPsdFy * rootPsdFy +
                        rootPsdFz * rootPsdFz);
        final double normRootPsdW = Math.sqrt(
                rootPsdWx * rootPsdWx + rootPsdWy * rootPsdWy +
                        rootPsdWz * rootPsdWz);

        assertEquals(avgPsdF, estimator.getAvgSpecificForceNoisePsd(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(normRootPsdF, estimator.getSpecificForceNoiseRootPsdNorm(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(avgPsdW, estimator.getAvgAngularRateNoisePsd(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(normRootPsdW, estimator.getAngularRateNoiseRootPsdNorm(),
                SMALL_ABSOLUTE_ERROR);

        assertEquals(rootPsdFx, accelNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdFy, accelNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdFz, accelNoiseRootPSD, ABSOLUTE_ERROR);

        assertEquals(rootPsdWx, gyroNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdWy, gyroNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdWz, gyroNoiseRootPSD, ABSOLUTE_ERROR);

        final double accelNoisePsd = accelNoiseRootPSD * accelNoiseRootPSD;
        assertEquals(estimator.getAvgSpecificForceNoisePsd(),
                accelNoisePsd, ABSOLUTE_ERROR);

        final double gyroNoisePsd = gyroNoiseRootPSD * gyroNoiseRootPSD;
        assertEquals(estimator.getAvgAngularRateNoisePsd(),
                gyroNoisePsd, ABSOLUTE_ERROR);

        // reset
        estimator.reset();

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertFalse(estimator.isRunning());
        assertEquals(mReset, 1);
    }

    @Test
    public void testAddBodyKinematicsAndReset2()
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

        final AccumulatedBodyKinematicsNoiseEstimator estimator =
                new AccumulatedBodyKinematicsNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mBodyKinematicsAdded, 0);
        assertEquals(mReset, 0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertFalse(estimator.isRunning());

        final BodyKinematics kinematics = new BodyKinematics();
        final double timeInterval = estimator.getTimeInterval();
        BodyKinematics lastKinematics = new BodyKinematics();

        double avgFx = 0.0;
        double avgFy = 0.0;
        double avgFz = 0.0;
        double avgWx = 0.0;
        double avgWy = 0.0;
        double avgWz = 0.0;
        double varFx = 0.0;
        double varFy = 0.0;
        double varFz = 0.0;
        double varWx = 0.0;
        double varWy = 0.0;
        double varWz = 0.0;
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastBodyKinematics(lastKinematics)) {
                assertEquals(estimator.getLastBodyKinematics(), lastKinematics);
                assertEquals(lastKinematics, kinematics);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                    errors, random, kinematics);

            final double fxi = kinematics.getFx();
            final double fyi = kinematics.getFy();
            final double fzi = kinematics.getFz();
            final double wxi = kinematics.getAngularRateX();
            final double wyi = kinematics.getAngularRateY();
            final double wzi = kinematics.getAngularRateZ();

            estimator.addBodyKinematics(
                    kinematics.getSpecificForceX(),
                    kinematics.getSpecificForceY(),
                    kinematics.getSpecificForceZ(),
                    kinematics.getAngularSpeedX(),
                    kinematics.getAngularSpeedY(),
                    kinematics.getAngularSpeedZ());

            assertTrue(estimator.getLastBodyKinematics(lastKinematics));
            assertEquals(lastKinematics, kinematics);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avgFx = avgFx * (double) i / (double) j + fxi / j;
            avgFy = avgFy * (double) i / (double) j + fyi / j;
            avgFz = avgFz * (double) i / (double) j + fzi / j;

            avgWx = avgWx * (double) i / (double) j + wxi / j;
            avgWy = avgWy * (double) i / (double) j + wyi / j;
            avgWz = avgWz * (double) i / (double) j + wzi / j;

            double diff = fxi - avgFx;
            double diff2 = diff * diff;
            varFx = varFx * (double) i / (double) j + diff2 / j;

            diff = fyi - avgFy;
            diff2 = diff * diff;
            varFy = varFy * (double) i / (double) j + diff2 / j;

            diff = fzi - avgFz;
            diff2 = diff * diff;
            varFz = varFz * (double) i / (double) j + diff2 / j;

            diff = wxi - avgWx;
            diff2 = diff * diff;
            varWx = varWx * (double) i / (double) j + diff2 / j;

            diff = wyi - avgWy;
            diff2 = diff * diff;
            varWy = varWy * (double) i / (double) j + diff2 / j;

            diff = wzi - avgWz;
            diff2 = diff * diff;
            varWz = varWz * (double) i / (double) j + diff2 / j;
        }

        assertEquals(estimator.getNumberOfProcessedSamples(), N_SAMPLES);
        assertFalse(estimator.isRunning());
        assertEquals(mStart, 1);
        assertEquals(mBodyKinematicsAdded, N_SAMPLES);
        assertEquals(mReset, 0);

        final double avgFxB = estimator.getAvgSpecificForceX();
        final double avgFyB = estimator.getAvgSpecificForceY();
        final double avgFzB = estimator.getAvgSpecificForceZ();

        final double avgWxB = estimator.getAvgAngularRateX();
        final double avgWyB = estimator.getAvgAngularRateY();
        final double avgWzB = estimator.getAvgAngularRateZ();

        assertEquals(avgFx, avgFxB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFy, avgFyB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFz, avgFzB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWx, avgWxB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWy, avgWyB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWz, avgWzB, SMALL_ABSOLUTE_ERROR);

        assertEquals(avgFxB, fx, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgFyB, fy, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgFzB, fz, LARGE_ABSOLUTE_ERROR);

        assertEquals(avgWxB, omegaX, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgWyB, omegaY, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgWzB, omegaZ, LARGE_ABSOLUTE_ERROR);

        final Acceleration avgFx1 = estimator.getAvgSpecificForceXAsMeasurement();
        final Acceleration avgFx2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceXAsMeasurement(avgFx2);

        assertEquals(avgFx, avgFx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFx1.getUnit());
        assertEquals(avgFx1, avgFx2);

        final Acceleration avgFy1 = estimator.getAvgSpecificForceYAsMeasurement();
        final Acceleration avgFy2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceYAsMeasurement(avgFy2);

        assertEquals(avgFy, avgFy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFy1.getUnit());
        assertEquals(avgFy1, avgFy2);

        final Acceleration avgFz1 = estimator.getAvgSpecificForceZAsMeasurement();
        final Acceleration avgFz2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceZAsMeasurement(avgFz2);

        assertEquals(avgFz, avgFz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFz1.getUnit());
        assertEquals(avgFz1, avgFz2);

        final AccelerationTriad avgFTriad1 = estimator.getAvgSpecificForceAsTriad();
        assertEquals(avgFx, avgFTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFy, avgFTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFz, avgFTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFTriad1.getUnit());
        final AccelerationTriad avgFTriad2 = new AccelerationTriad();
        estimator.getAvgSpecificForceAsTriad(avgFTriad2);
        assertEquals(avgFTriad1, avgFTriad2);
        assertEquals(avgFTriad1.getNorm(), estimator.getAvgSpecificForceNorm(),
                0.0);
        final Acceleration avgFNorm1 = estimator.getAvgSpecificForceNormAsMeasurement();
        assertEquals(estimator.getAvgSpecificForceNorm(),
                avgFNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFNorm1.getUnit());
        final Acceleration avgFNorm2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceNormAsMeasurement(avgFNorm2);
        assertEquals(avgFNorm1, avgFNorm2);

        final AngularSpeed avgWx1 = estimator.getAvgAngularRateXAsMeasurement();
        assertEquals(avgWx, avgWx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWx1.getUnit());
        final AngularSpeed avgWx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateXAsMeasurement(avgWx2);
        assertEquals(avgWx1, avgWx2);

        final AngularSpeed avgWy1 = estimator.getAvgAngularRateYAsMeasurement();
        assertEquals(avgWy, avgWy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWy1.getUnit());
        final AngularSpeed avgWy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateYAsMeasurement(avgWy2);
        assertEquals(avgWy1, avgWy2);

        final AngularSpeed avgWz1 = estimator.getAvgAngularRateZAsMeasurement();
        assertEquals(avgWz, avgWz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWz1.getUnit());
        final AngularSpeed avgWz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateZAsMeasurement(avgWz2);
        assertEquals(avgWz1, avgWz2);

        final AngularSpeedTriad avgWTriad1 = estimator.getAvgAngularRateTriad();
        assertEquals(avgWx, avgWTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWy, avgWTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWz, avgWTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWTriad1.getUnit());
        final AngularSpeedTriad avgWTriad2 = new AngularSpeedTriad();
        estimator.getAvgAngularRateTriad(avgWTriad2);
        assertEquals(avgWTriad1, avgWTriad2);

        assertEquals(avgWTriad1.getNorm(), estimator.getAvgAngularRateNorm(),
                0.0);
        final AngularSpeed avgWNorm1 = estimator.getAvgAngularRateNormAsMeasurement();
        assertEquals(estimator.getAvgAngularRateNorm(),
                avgWNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWNorm1.getUnit());
        final AngularSpeed avgWNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAvgAngularRateNormAsMeasurement(avgWNorm2);
        assertEquals(avgWNorm1, avgWNorm2);

        final BodyKinematics avgKinematics1 = estimator.getAvgBodyKinematics();
        assertEquals(avgFx, avgKinematics1.getFx(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFy, avgKinematics1.getFy(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFz, avgKinematics1.getFz(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWx, avgKinematics1.getAngularRateX(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWy, avgKinematics1.getAngularRateY(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWz, avgKinematics1.getAngularRateZ(),
                SMALL_ABSOLUTE_ERROR);
        final BodyKinematics avgKinematics2 = new BodyKinematics();
        estimator.getAvgBodyKinematics(avgKinematics2);
        assertEquals(avgKinematics1, avgKinematics2);

        assertEquals(varFx, estimator.getVarianceSpecificForceX(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(varFy, estimator.getVarianceSpecificForceY(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(varFz, estimator.getVarianceSpecificForceZ(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(varWx, estimator.getVarianceAngularRateX(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(varWy, estimator.getVarianceAngularRateY(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(varWz, estimator.getVarianceAngularRateZ(),
                SMALL_ABSOLUTE_ERROR);

        final double stdFx = Math.sqrt(varFx);
        final double stdFy = Math.sqrt(varFy);
        final double stdFz = Math.sqrt(varFz);
        final double stdWx = Math.sqrt(varWx);
        final double stdWy = Math.sqrt(varWy);
        final double stdWz = Math.sqrt(varWz);

        assertEquals(stdFx, estimator.getStandardDeviationSpecificForceX(),
                SMALL_ABSOLUTE_ERROR);
        final Acceleration stdFx1 = estimator.getStandardDeviationSpecificForceXAsMeasurement();
        assertEquals(stdFx, stdFx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                stdFx1.getUnit());
        final Acceleration stdFx2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceXAsMeasurement(stdFx2);
        assertEquals(stdFx1, stdFx2);

        assertEquals(stdFy, estimator.getStandardDeviationSpecificForceY(),
                SMALL_ABSOLUTE_ERROR);
        final Acceleration stdFy1 = estimator.getStandardDeviationSpecificForceYAsMeasurement();
        assertEquals(stdFy, stdFy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                stdFy1.getUnit());
        final Acceleration stdFy2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceYAsMeasurement(stdFy2);
        assertEquals(stdFy1, stdFy2);

        assertEquals(stdFz, estimator.getStandardDeviationSpecificForceZ(),
                SMALL_ABSOLUTE_ERROR);
        final Acceleration stdFz1 = estimator.getStandardDeviationSpecificForceZAsMeasurement();
        assertEquals(stdFz, stdFz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                stdFz1.getUnit());
        final Acceleration stdFz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceZAsMeasurement(stdFz2);
        assertEquals(stdFz1, stdFz2);

        final AccelerationTriad stdFTriad1 = estimator.getStandardDeviationSpecificForceTriad();
        assertEquals(stdFx, stdFTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFy, stdFTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFz, stdFTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                stdFTriad1.getUnit());

        final AccelerationTriad stdFTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationSpecificForceTriad(stdFTriad2);
        assertEquals(stdFTriad1, stdFTriad2);

        assertEquals(stdFTriad1.getNorm(),
                estimator.getStandardDeviationSpecificForceNorm(),
                SMALL_ABSOLUTE_ERROR);
        final Acceleration stdFNorm1 = estimator.getStandardDeviationSpecificForceNormAsMeasurement();
        assertEquals(stdFTriad1.getNorm(), stdFNorm1.getValue().doubleValue(),
                0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                stdFNorm1.getUnit());
        final Acceleration stdFNorm2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceNormAsMeasurement(stdFNorm2);
        assertEquals(stdFNorm1, stdFNorm2);

        final double avgStdF = (stdFx + stdFy + stdFz) / 3.0;
        assertEquals(avgStdF, estimator.getAverageStandardDeviationSpecificForce(),
                SMALL_ABSOLUTE_ERROR);
        final Acceleration avgStdF1 = estimator.getAverageStandardDeviationSpecificForceAsMeasurement();
        assertEquals(avgStdF, avgStdF1.getValue().doubleValue(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgStdF1.getUnit());
        final Acceleration avgStdF2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAverageStandardDeviationSpecificForceAsMeasurement(avgStdF2);
        assertEquals(avgStdF1, avgStdF2);

        assertEquals(stdWx, estimator.getStandardDeviationAngularRateX(),
                SMALL_ABSOLUTE_ERROR);
        final AngularSpeed stdWx1 = estimator.getStandardDeviationAngularRateXAsMeasurement();
        assertEquals(stdWx, stdWx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWx1.getUnit());
        final AngularSpeed stdWx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateXAsMeasurement(stdWx2);
        assertEquals(stdWx1, stdWx2);

        assertEquals(stdWy, estimator.getStandardDeviationAngularRateY(),
                SMALL_ABSOLUTE_ERROR);
        final AngularSpeed stdWy1 = estimator.getStandardDeviationAngularRateYAsMeasurement();
        assertEquals(stdWy, stdWy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWy1.getUnit());
        final AngularSpeed stdWy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateYAsMeasurement(stdWy2);
        assertEquals(stdWy1, stdWy2);

        assertEquals(stdWz, estimator.getStandardDeviationAngularRateZ(),
                SMALL_ABSOLUTE_ERROR);
        final AngularSpeed stdWz1 = estimator.getStandardDeviationAngularRateZAsMeasurement();
        assertEquals(stdWz, stdWz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWz1.getUnit());
        final AngularSpeed stdWz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateZAsMeasurement(stdWz2);
        assertEquals(stdWz1, stdWz2);

        final AngularSpeedTriad stdWTriad1 = estimator.getStandardDeviationAngularSpeedTriad();
        assertEquals(stdWx, stdWTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWy, stdWTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWz, stdWTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWTriad1.getUnit());
        final AngularSpeedTriad stdWTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularSpeedTriad(stdWTriad2);
        assertEquals(stdWTriad1, stdWTriad2);

        assertEquals(stdWTriad1.getNorm(), estimator.getStandardDeviationAngularSpeedNorm(),
                0.0);
        final AngularSpeed stdWNorm1 = estimator.getStandardDeviationAngularSpeedNormAsMeasurement();
        assertEquals(stdWTriad1.getNorm(), stdWNorm1.getValue().doubleValue(),
                0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWNorm1.getUnit());
        final AngularSpeed stdWNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularSpeedNormAsMeasurement(stdWNorm2);
        assertEquals(stdWNorm1, stdWNorm2);

        final double avgStdW = (stdWx + stdWy + stdWz) / 3.0;
        assertEquals(avgStdW, estimator.getAverageStandardDeviationAngularSpeed(),
                SMALL_ABSOLUTE_ERROR);
        final AngularSpeed avgStdW1 = estimator.getAverageStandardDeviationAngularSpeedAsMeasurement();
        assertEquals(avgStdW, avgStdW1.getValue().doubleValue(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgStdW1.getUnit());
        final AngularSpeed avgStdW2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAverageStandardDeviationAngularSpeedAsMeasurement(avgStdW2);
        assertEquals(avgStdW1, avgStdW2);

        final BodyKinematics stdKinematics1 = estimator.getStandardDeviationAsBodyKinematics();
        assertEquals(stdFx, stdKinematics1.getFx(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFy, stdKinematics1.getFy(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFz, stdKinematics1.getFz(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWx, stdKinematics1.getAngularRateX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWy, stdKinematics1.getAngularRateY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWz, stdKinematics1.getAngularRateZ(), SMALL_ABSOLUTE_ERROR);
        final BodyKinematics stdKinematics2 = new BodyKinematics();
        estimator.getStandardDeviationAsBodyKinematics(stdKinematics2);
        assertEquals(stdKinematics1, stdKinematics2);

        final double psdFx = estimator.getSpecificForcePsdX();
        final double psdFy = estimator.getSpecificForcePsdY();
        final double psdFz = estimator.getSpecificForcePsdZ();
        final double psdWx = estimator.getAngularRatePsdX();
        final double psdWy = estimator.getAngularRatePsdY();
        final double psdWz = estimator.getAngularRatePsdZ();

        assertEquals(psdFx, varFx * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdFy, varFy * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdFz, varFz * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdWx, varWx * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdWy, varWy * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdWz, varWz * timeInterval, SMALL_ABSOLUTE_ERROR);

        final double rootPsdFx = Math.sqrt(psdFx);
        final double rootPsdFy = Math.sqrt(psdFy);
        final double rootPsdFz = Math.sqrt(psdFz);
        final double rootPsdWx = Math.sqrt(psdWx);
        final double rootPsdWy = Math.sqrt(psdWy);
        final double rootPsdWz = Math.sqrt(psdWz);

        assertEquals(rootPsdFx, estimator.getSpecificForceRootPsdX(),
                0.0);
        assertEquals(rootPsdFy, estimator.getSpecificForceRootPsdY(),
                0.0);
        assertEquals(rootPsdFz, estimator.getSpecificForceRootPsdZ(),
                0.0);
        assertEquals(rootPsdWx, estimator.getAngularRateRootPsdX(),
                0.0);
        assertEquals(rootPsdWy, estimator.getAngularRateRootPsdY(),
                0.0);
        assertEquals(rootPsdWz, estimator.getAngularRateRootPsdZ(),
                0.0);

        final double avgPsdF = (psdFx + psdFy + psdFz) / 3.0;
        final double avgPsdW = (psdWx + psdWy + psdWz) / 3.0;
        final double normRootPsdF = Math.sqrt(
                rootPsdFx * rootPsdFx + rootPsdFy * rootPsdFy +
                        rootPsdFz * rootPsdFz);
        final double normRootPsdW = Math.sqrt(
                rootPsdWx * rootPsdWx + rootPsdWy * rootPsdWy +
                        rootPsdWz * rootPsdWz);

        assertEquals(avgPsdF, estimator.getAvgSpecificForceNoisePsd(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(normRootPsdF, estimator.getSpecificForceNoiseRootPsdNorm(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(avgPsdW, estimator.getAvgAngularRateNoisePsd(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(normRootPsdW, estimator.getAngularRateNoiseRootPsdNorm(),
                SMALL_ABSOLUTE_ERROR);

        assertEquals(rootPsdFx, accelNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdFy, accelNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdFz, accelNoiseRootPSD, ABSOLUTE_ERROR);

        assertEquals(rootPsdWx, gyroNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdWy, gyroNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdWz, gyroNoiseRootPSD, ABSOLUTE_ERROR);

        final double accelNoisePsd = accelNoiseRootPSD * accelNoiseRootPSD;
        assertEquals(estimator.getAvgSpecificForceNoisePsd(),
                accelNoisePsd, ABSOLUTE_ERROR);

        final double gyroNoisePsd = gyroNoiseRootPSD * gyroNoiseRootPSD;
        assertEquals(estimator.getAvgAngularRateNoisePsd(),
                gyroNoisePsd, ABSOLUTE_ERROR);

        // reset
        estimator.reset();

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertFalse(estimator.isRunning());
        assertEquals(mReset, 1);
    }

    @Test
    public void testAddBodyKinematicsAndReset3()
            throws WrongSizeException, LockedException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
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

            final AccumulatedBodyKinematicsNoiseEstimator estimator =
                    new AccumulatedBodyKinematicsNoiseEstimator(this);

            reset();
            assertEquals(mStart, 0);
            assertEquals(mBodyKinematicsAdded, 0);
            assertEquals(mReset, 0);
            assertEquals(estimator.getNumberOfProcessedSamples(), 0);
            assertNull(estimator.getLastBodyKinematics());
            assertFalse(estimator.getLastBodyKinematics(null));
            assertFalse(estimator.isRunning());

            final BodyKinematics kinematics = new BodyKinematics();
            final double timeInterval = estimator.getTimeInterval();
            BodyKinematics lastKinematics = new BodyKinematics();

            double avgFx = 0.0;
            double avgFy = 0.0;
            double avgFz = 0.0;
            double avgWx = 0.0;
            double avgWy = 0.0;
            double avgWz = 0.0;
            double varFx = 0.0;
            double varFy = 0.0;
            double varFz = 0.0;
            double varWx = 0.0;
            double varWy = 0.0;
            double varWz = 0.0;
            for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
                if (estimator.getLastBodyKinematics(lastKinematics)) {
                    assertEquals(estimator.getLastBodyKinematics(), lastKinematics);
                    assertEquals(lastKinematics, kinematics);
                }

                BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                        errors, random, kinematics);

                final double fxi = kinematics.getFx();
                final double fyi = kinematics.getFy();
                final double fzi = kinematics.getFz();
                final double wxi = kinematics.getAngularRateX();
                final double wyi = kinematics.getAngularRateY();
                final double wzi = kinematics.getAngularRateZ();

                estimator.addBodyKinematics(
                        kinematics.getSpecificForceTriad(),
                        kinematics.getAngularRateTriad());

                assertTrue(estimator.getLastBodyKinematics(lastKinematics));
                assertEquals(lastKinematics, kinematics);
                assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
                assertFalse(estimator.isRunning());

                avgFx = avgFx * (double) i / (double) j + fxi / j;
                avgFy = avgFy * (double) i / (double) j + fyi / j;
                avgFz = avgFz * (double) i / (double) j + fzi / j;

                avgWx = avgWx * (double) i / (double) j + wxi / j;
                avgWy = avgWy * (double) i / (double) j + wyi / j;
                avgWz = avgWz * (double) i / (double) j + wzi / j;

                double diff = fxi - avgFx;
                double diff2 = diff * diff;
                varFx = varFx * (double) i / (double) j + diff2 / j;

                diff = fyi - avgFy;
                diff2 = diff * diff;
                varFy = varFy * (double) i / (double) j + diff2 / j;

                diff = fzi - avgFz;
                diff2 = diff * diff;
                varFz = varFz * (double) i / (double) j + diff2 / j;

                diff = wxi - avgWx;
                diff2 = diff * diff;
                varWx = varWx * (double) i / (double) j + diff2 / j;

                diff = wyi - avgWy;
                diff2 = diff * diff;
                varWy = varWy * (double) i / (double) j + diff2 / j;

                diff = wzi - avgWz;
                diff2 = diff * diff;
                varWz = varWz * (double) i / (double) j + diff2 / j;
            }

            assertEquals(estimator.getNumberOfProcessedSamples(), N_SAMPLES);
            assertFalse(estimator.isRunning());
            assertEquals(mStart, 1);
            assertEquals(mBodyKinematicsAdded, N_SAMPLES);
            assertEquals(mReset, 0);

            final double avgFxB = estimator.getAvgSpecificForceX();
            final double avgFyB = estimator.getAvgSpecificForceY();
            final double avgFzB = estimator.getAvgSpecificForceZ();

            final double avgWxB = estimator.getAvgAngularRateX();
            final double avgWyB = estimator.getAvgAngularRateY();
            final double avgWzB = estimator.getAvgAngularRateZ();

            if (Math.abs(avgFx - avgFxB) > SMALL_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(avgFy - avgFyB) > SMALL_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(avgFz - avgFzB) > SMALL_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(avgWx - avgWxB) > SMALL_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(avgWy - avgWyB) > SMALL_ABSOLUTE_ERROR) {
                continue;
            }
            if (Math.abs(avgWz - avgWzB) > SMALL_ABSOLUTE_ERROR) {
                continue;
            }
            assertEquals(avgFx, avgFxB, SMALL_ABSOLUTE_ERROR);
            assertEquals(avgFy, avgFyB, SMALL_ABSOLUTE_ERROR);
            assertEquals(avgFz, avgFzB, SMALL_ABSOLUTE_ERROR);
            assertEquals(avgWx, avgWxB, SMALL_ABSOLUTE_ERROR);
            assertEquals(avgWy, avgWyB, SMALL_ABSOLUTE_ERROR);
            assertEquals(avgWz, avgWzB, SMALL_ABSOLUTE_ERROR);

            assertEquals(avgFxB, fx, LARGE_ABSOLUTE_ERROR);
            assertEquals(avgFyB, fy, LARGE_ABSOLUTE_ERROR);
            assertEquals(avgFzB, fz, LARGE_ABSOLUTE_ERROR);

            assertEquals(avgWxB, omegaX, LARGE_ABSOLUTE_ERROR);
            assertEquals(avgWyB, omegaY, LARGE_ABSOLUTE_ERROR);
            assertEquals(avgWzB, omegaZ, LARGE_ABSOLUTE_ERROR);

            final Acceleration avgFx1 = estimator.getAvgSpecificForceXAsMeasurement();
            final Acceleration avgFx2 = new Acceleration(1.0,
                    AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getAvgSpecificForceXAsMeasurement(avgFx2);

            assertEquals(avgFx, avgFx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFx1.getUnit());
            assertEquals(avgFx1, avgFx2);

            final Acceleration avgFy1 = estimator.getAvgSpecificForceYAsMeasurement();
            final Acceleration avgFy2 = new Acceleration(1.0,
                    AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getAvgSpecificForceYAsMeasurement(avgFy2);

            assertEquals(avgFy, avgFy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFy1.getUnit());
            assertEquals(avgFy1, avgFy2);

            final Acceleration avgFz1 = estimator.getAvgSpecificForceZAsMeasurement();
            final Acceleration avgFz2 = new Acceleration(1.0,
                    AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getAvgSpecificForceZAsMeasurement(avgFz2);

            assertEquals(avgFz, avgFz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFz1.getUnit());
            assertEquals(avgFz1, avgFz2);

            final AccelerationTriad avgFTriad1 = estimator.getAvgSpecificForceAsTriad();
            assertEquals(avgFx, avgFTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
            assertEquals(avgFy, avgFTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
            assertEquals(avgFz, avgFTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFTriad1.getUnit());
            final AccelerationTriad avgFTriad2 = new AccelerationTriad();
            estimator.getAvgSpecificForceAsTriad(avgFTriad2);
            assertEquals(avgFTriad1, avgFTriad2);
            assertEquals(avgFTriad1.getNorm(), estimator.getAvgSpecificForceNorm(),
                    0.0);
            final Acceleration avgFNorm1 = estimator.getAvgSpecificForceNormAsMeasurement();
            assertEquals(estimator.getAvgSpecificForceNorm(),
                    avgFNorm1.getValue().doubleValue(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFNorm1.getUnit());
            final Acceleration avgFNorm2 = new Acceleration(1.0,
                    AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getAvgSpecificForceNormAsMeasurement(avgFNorm2);
            assertEquals(avgFNorm1, avgFNorm2);

            final AngularSpeed avgWx1 = estimator.getAvgAngularRateXAsMeasurement();
            assertEquals(avgWx, avgWx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWx1.getUnit());
            final AngularSpeed avgWx2 = new AngularSpeed(
                    1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getAvgAngularRateXAsMeasurement(avgWx2);
            assertEquals(avgWx1, avgWx2);

            final AngularSpeed avgWy1 = estimator.getAvgAngularRateYAsMeasurement();
            assertEquals(avgWy, avgWy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWy1.getUnit());
            final AngularSpeed avgWy2 = new AngularSpeed(
                    1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getAvgAngularRateYAsMeasurement(avgWy2);
            assertEquals(avgWy1, avgWy2);

            final AngularSpeed avgWz1 = estimator.getAvgAngularRateZAsMeasurement();
            assertEquals(avgWz, avgWz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWz1.getUnit());
            final AngularSpeed avgWz2 = new AngularSpeed(
                    1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getAvgAngularRateZAsMeasurement(avgWz2);
            assertEquals(avgWz1, avgWz2);

            final AngularSpeedTriad avgWTriad1 = estimator.getAvgAngularRateTriad();
            assertEquals(avgWx, avgWTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
            assertEquals(avgWy, avgWTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
            assertEquals(avgWz, avgWTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWTriad1.getUnit());
            final AngularSpeedTriad avgWTriad2 = new AngularSpeedTriad();
            estimator.getAvgAngularRateTriad(avgWTriad2);
            assertEquals(avgWTriad1, avgWTriad2);

            assertEquals(avgWTriad1.getNorm(), estimator.getAvgAngularRateNorm(),
                    0.0);
            final AngularSpeed avgWNorm1 = estimator.getAvgAngularRateNormAsMeasurement();
            assertEquals(estimator.getAvgAngularRateNorm(),
                    avgWNorm1.getValue().doubleValue(), 0.0);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWNorm1.getUnit());
            final AngularSpeed avgWNorm2 = new AngularSpeed(
                    1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
            estimator.getAvgAngularRateNormAsMeasurement(avgWNorm2);
            assertEquals(avgWNorm1, avgWNorm2);

            final BodyKinematics avgKinematics1 = estimator.getAvgBodyKinematics();
            assertEquals(avgFx, avgKinematics1.getFx(), SMALL_ABSOLUTE_ERROR);
            assertEquals(avgFy, avgKinematics1.getFy(), SMALL_ABSOLUTE_ERROR);
            assertEquals(avgFz, avgKinematics1.getFz(), SMALL_ABSOLUTE_ERROR);
            assertEquals(avgWx, avgKinematics1.getAngularRateX(),
                    SMALL_ABSOLUTE_ERROR);
            assertEquals(avgWy, avgKinematics1.getAngularRateY(),
                    SMALL_ABSOLUTE_ERROR);
            assertEquals(avgWz, avgKinematics1.getAngularRateZ(),
                    SMALL_ABSOLUTE_ERROR);
            final BodyKinematics avgKinematics2 = new BodyKinematics();
            estimator.getAvgBodyKinematics(avgKinematics2);
            assertEquals(avgKinematics1, avgKinematics2);

            assertEquals(varFx, estimator.getVarianceSpecificForceX(),
                    SMALL_ABSOLUTE_ERROR);
            assertEquals(varFy, estimator.getVarianceSpecificForceY(),
                    SMALL_ABSOLUTE_ERROR);
            assertEquals(varFz, estimator.getVarianceSpecificForceZ(),
                    SMALL_ABSOLUTE_ERROR);
            assertEquals(varWx, estimator.getVarianceAngularRateX(),
                    SMALL_ABSOLUTE_ERROR);
            assertEquals(varWy, estimator.getVarianceAngularRateY(),
                    SMALL_ABSOLUTE_ERROR);
            assertEquals(varWz, estimator.getVarianceAngularRateZ(),
                    SMALL_ABSOLUTE_ERROR);

            final double stdFx = Math.sqrt(varFx);
            final double stdFy = Math.sqrt(varFy);
            final double stdFz = Math.sqrt(varFz);
            final double stdWx = Math.sqrt(varWx);
            final double stdWy = Math.sqrt(varWy);
            final double stdWz = Math.sqrt(varWz);

            assertEquals(stdFx, estimator.getStandardDeviationSpecificForceX(),
                    SMALL_ABSOLUTE_ERROR);
            final Acceleration stdFx1 = estimator.getStandardDeviationSpecificForceXAsMeasurement();
            assertEquals(stdFx, stdFx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                    stdFx1.getUnit());
            final Acceleration stdFx2 = new Acceleration(
                    1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getStandardDeviationSpecificForceXAsMeasurement(stdFx2);
            assertEquals(stdFx1, stdFx2);

            assertEquals(stdFy, estimator.getStandardDeviationSpecificForceY(),
                    SMALL_ABSOLUTE_ERROR);
            final Acceleration stdFy1 = estimator.getStandardDeviationSpecificForceYAsMeasurement();
            assertEquals(stdFy, stdFy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                    stdFy1.getUnit());
            final Acceleration stdFy2 = new Acceleration(
                    1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getStandardDeviationSpecificForceYAsMeasurement(stdFy2);
            assertEquals(stdFy1, stdFy2);

            assertEquals(stdFz, estimator.getStandardDeviationSpecificForceZ(),
                    SMALL_ABSOLUTE_ERROR);
            final Acceleration stdFz1 = estimator.getStandardDeviationSpecificForceZAsMeasurement();
            assertEquals(stdFz, stdFz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                    stdFz1.getUnit());
            final Acceleration stdFz2 = new Acceleration(
                    1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getStandardDeviationSpecificForceZAsMeasurement(stdFz2);
            assertEquals(stdFz1, stdFz2);

            final AccelerationTriad stdFTriad1 = estimator.getStandardDeviationSpecificForceTriad();
            assertEquals(stdFx, stdFTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
            assertEquals(stdFy, stdFTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
            assertEquals(stdFz, stdFTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                    stdFTriad1.getUnit());

            final AccelerationTriad stdFTriad2 = new AccelerationTriad();
            estimator.getStandardDeviationSpecificForceTriad(stdFTriad2);
            assertEquals(stdFTriad1, stdFTriad2);

            assertEquals(stdFTriad1.getNorm(),
                    estimator.getStandardDeviationSpecificForceNorm(),
                    SMALL_ABSOLUTE_ERROR);
            final Acceleration stdFNorm1 = estimator.getStandardDeviationSpecificForceNormAsMeasurement();
            assertEquals(stdFTriad1.getNorm(), stdFNorm1.getValue().doubleValue(),
                    0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                    stdFNorm1.getUnit());
            final Acceleration stdFNorm2 = new Acceleration(
                    1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getStandardDeviationSpecificForceNormAsMeasurement(stdFNorm2);
            assertEquals(stdFNorm1, stdFNorm2);

            final double avgStdF = (stdFx + stdFy + stdFz) / 3.0;
            assertEquals(avgStdF, estimator.getAverageStandardDeviationSpecificForce(),
                    SMALL_ABSOLUTE_ERROR);
            final Acceleration avgStdF1 = estimator.getAverageStandardDeviationSpecificForceAsMeasurement();
            assertEquals(avgStdF, avgStdF1.getValue().doubleValue(),
                    SMALL_ABSOLUTE_ERROR);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgStdF1.getUnit());
            final Acceleration avgStdF2 = new Acceleration(
                    1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getAverageStandardDeviationSpecificForceAsMeasurement(avgStdF2);
            assertEquals(avgStdF1, avgStdF2);

            assertEquals(stdWx, estimator.getStandardDeviationAngularRateX(),
                    SMALL_ABSOLUTE_ERROR);
            final AngularSpeed stdWx1 = estimator.getStandardDeviationAngularRateXAsMeasurement();
            assertEquals(stdWx, stdWx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWx1.getUnit());
            final AngularSpeed stdWx2 = new AngularSpeed(
                    1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getStandardDeviationAngularRateXAsMeasurement(stdWx2);
            assertEquals(stdWx1, stdWx2);

            assertEquals(stdWy, estimator.getStandardDeviationAngularRateY(),
                    SMALL_ABSOLUTE_ERROR);
            final AngularSpeed stdWy1 = estimator.getStandardDeviationAngularRateYAsMeasurement();
            assertEquals(stdWy, stdWy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWy1.getUnit());
            final AngularSpeed stdWy2 = new AngularSpeed(
                    1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getStandardDeviationAngularRateYAsMeasurement(stdWy2);
            assertEquals(stdWy1, stdWy2);

            assertEquals(stdWz, estimator.getStandardDeviationAngularRateZ(),
                    SMALL_ABSOLUTE_ERROR);
            final AngularSpeed stdWz1 = estimator.getStandardDeviationAngularRateZAsMeasurement();
            assertEquals(stdWz, stdWz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWz1.getUnit());
            final AngularSpeed stdWz2 = new AngularSpeed(
                    1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getStandardDeviationAngularRateZAsMeasurement(stdWz2);
            assertEquals(stdWz1, stdWz2);

            final AngularSpeedTriad stdWTriad1 = estimator.getStandardDeviationAngularSpeedTriad();
            assertEquals(stdWx, stdWTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
            assertEquals(stdWy, stdWTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
            assertEquals(stdWz, stdWTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWTriad1.getUnit());
            final AngularSpeedTriad stdWTriad2 = new AngularSpeedTriad();
            estimator.getStandardDeviationAngularSpeedTriad(stdWTriad2);
            assertEquals(stdWTriad1, stdWTriad2);

            assertEquals(stdWTriad1.getNorm(), estimator.getStandardDeviationAngularSpeedNorm(),
                    0.0);
            final AngularSpeed stdWNorm1 = estimator.getStandardDeviationAngularSpeedNormAsMeasurement();
            assertEquals(stdWTriad1.getNorm(), stdWNorm1.getValue().doubleValue(),
                    0.0);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWNorm1.getUnit());
            final AngularSpeed stdWNorm2 = new AngularSpeed(
                    1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getStandardDeviationAngularSpeedNormAsMeasurement(stdWNorm2);
            assertEquals(stdWNorm1, stdWNorm2);

            final double avgStdW = (stdWx + stdWy + stdWz) / 3.0;
            assertEquals(avgStdW, estimator.getAverageStandardDeviationAngularSpeed(),
                    SMALL_ABSOLUTE_ERROR);
            final AngularSpeed avgStdW1 = estimator.getAverageStandardDeviationAngularSpeedAsMeasurement();
            assertEquals(avgStdW, avgStdW1.getValue().doubleValue(),
                    SMALL_ABSOLUTE_ERROR);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgStdW1.getUnit());
            final AngularSpeed avgStdW2 = new AngularSpeed(
                    1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getAverageStandardDeviationAngularSpeedAsMeasurement(avgStdW2);
            assertEquals(avgStdW1, avgStdW2);

            final BodyKinematics stdKinematics1 = estimator.getStandardDeviationAsBodyKinematics();
            assertEquals(stdFx, stdKinematics1.getFx(), SMALL_ABSOLUTE_ERROR);
            assertEquals(stdFy, stdKinematics1.getFy(), SMALL_ABSOLUTE_ERROR);
            assertEquals(stdFz, stdKinematics1.getFz(), SMALL_ABSOLUTE_ERROR);
            assertEquals(stdWx, stdKinematics1.getAngularRateX(), SMALL_ABSOLUTE_ERROR);
            assertEquals(stdWy, stdKinematics1.getAngularRateY(), SMALL_ABSOLUTE_ERROR);
            assertEquals(stdWz, stdKinematics1.getAngularRateZ(), SMALL_ABSOLUTE_ERROR);
            final BodyKinematics stdKinematics2 = new BodyKinematics();
            estimator.getStandardDeviationAsBodyKinematics(stdKinematics2);
            assertEquals(stdKinematics1, stdKinematics2);

            final double psdFx = estimator.getSpecificForcePsdX();
            final double psdFy = estimator.getSpecificForcePsdY();
            final double psdFz = estimator.getSpecificForcePsdZ();
            final double psdWx = estimator.getAngularRatePsdX();
            final double psdWy = estimator.getAngularRatePsdY();
            final double psdWz = estimator.getAngularRatePsdZ();

            assertEquals(psdFx, varFx * timeInterval, SMALL_ABSOLUTE_ERROR);
            assertEquals(psdFy, varFy * timeInterval, SMALL_ABSOLUTE_ERROR);
            assertEquals(psdFz, varFz * timeInterval, SMALL_ABSOLUTE_ERROR);
            assertEquals(psdWx, varWx * timeInterval, SMALL_ABSOLUTE_ERROR);
            assertEquals(psdWy, varWy * timeInterval, SMALL_ABSOLUTE_ERROR);
            assertEquals(psdWz, varWz * timeInterval, SMALL_ABSOLUTE_ERROR);

            final double rootPsdFx = Math.sqrt(psdFx);
            final double rootPsdFy = Math.sqrt(psdFy);
            final double rootPsdFz = Math.sqrt(psdFz);
            final double rootPsdWx = Math.sqrt(psdWx);
            final double rootPsdWy = Math.sqrt(psdWy);
            final double rootPsdWz = Math.sqrt(psdWz);

            assertEquals(rootPsdFx, estimator.getSpecificForceRootPsdX(),
                    0.0);
            assertEquals(rootPsdFy, estimator.getSpecificForceRootPsdY(),
                    0.0);
            assertEquals(rootPsdFz, estimator.getSpecificForceRootPsdZ(),
                    0.0);
            assertEquals(rootPsdWx, estimator.getAngularRateRootPsdX(),
                    0.0);
            assertEquals(rootPsdWy, estimator.getAngularRateRootPsdY(),
                    0.0);
            assertEquals(rootPsdWz, estimator.getAngularRateRootPsdZ(),
                    0.0);

            final double avgPsdF = (psdFx + psdFy + psdFz) / 3.0;
            final double avgPsdW = (psdWx + psdWy + psdWz) / 3.0;
            final double normRootPsdF = Math.sqrt(
                    rootPsdFx * rootPsdFx + rootPsdFy * rootPsdFy +
                            rootPsdFz * rootPsdFz);
            final double normRootPsdW = Math.sqrt(
                    rootPsdWx * rootPsdWx + rootPsdWy * rootPsdWy +
                            rootPsdWz * rootPsdWz);

            assertEquals(avgPsdF, estimator.getAvgSpecificForceNoisePsd(),
                    SMALL_ABSOLUTE_ERROR);
            assertEquals(normRootPsdF, estimator.getSpecificForceNoiseRootPsdNorm(),
                    SMALL_ABSOLUTE_ERROR);
            assertEquals(avgPsdW, estimator.getAvgAngularRateNoisePsd(),
                    SMALL_ABSOLUTE_ERROR);
            assertEquals(normRootPsdW, estimator.getAngularRateNoiseRootPsdNorm(),
                    SMALL_ABSOLUTE_ERROR);

            assertEquals(rootPsdFx, accelNoiseRootPSD, ABSOLUTE_ERROR);
            assertEquals(rootPsdFy, accelNoiseRootPSD, ABSOLUTE_ERROR);
            assertEquals(rootPsdFz, accelNoiseRootPSD, ABSOLUTE_ERROR);

            assertEquals(rootPsdWx, gyroNoiseRootPSD, ABSOLUTE_ERROR);
            assertEquals(rootPsdWy, gyroNoiseRootPSD, ABSOLUTE_ERROR);
            assertEquals(rootPsdWz, gyroNoiseRootPSD, ABSOLUTE_ERROR);

            final double accelNoisePsd = accelNoiseRootPSD * accelNoiseRootPSD;
            assertEquals(estimator.getAvgSpecificForceNoisePsd(),
                    accelNoisePsd, ABSOLUTE_ERROR);

            final double gyroNoisePsd = gyroNoiseRootPSD * gyroNoiseRootPSD;
            assertEquals(estimator.getAvgAngularRateNoisePsd(),
                    gyroNoisePsd, ABSOLUTE_ERROR);

            // reset
            estimator.reset();

            assertEquals(0, estimator.getNumberOfProcessedSamples());
            assertNull(estimator.getLastBodyKinematics());
            assertFalse(estimator.getLastBodyKinematics(null));
            assertFalse(estimator.isRunning());
            assertEquals(mReset, 1);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testAddBodyKinematicsAndReset4()
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

        final AccumulatedBodyKinematicsNoiseEstimator estimator =
                new AccumulatedBodyKinematicsNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mBodyKinematicsAdded, 0);
        assertEquals(mReset, 0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertFalse(estimator.isRunning());

        final BodyKinematics kinematics = new BodyKinematics();
        final double timeInterval = estimator.getTimeInterval();
        BodyKinematics lastKinematics = new BodyKinematics();

        double avgFx = 0.0;
        double avgFy = 0.0;
        double avgFz = 0.0;
        double avgWx = 0.0;
        double avgWy = 0.0;
        double avgWz = 0.0;
        double varFx = 0.0;
        double varFy = 0.0;
        double varFz = 0.0;
        double varWx = 0.0;
        double varWy = 0.0;
        double varWz = 0.0;
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastBodyKinematics(lastKinematics)) {
                assertEquals(estimator.getLastBodyKinematics(), lastKinematics);
                assertEquals(lastKinematics, kinematics);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                    errors, random, kinematics);

            final double fxi = kinematics.getFx();
            final double fyi = kinematics.getFy();
            final double fzi = kinematics.getFz();
            final double wxi = kinematics.getAngularRateX();
            final double wyi = kinematics.getAngularRateY();
            final double wzi = kinematics.getAngularRateZ();

            estimator.addBodyKinematics(kinematics);

            assertTrue(estimator.getLastBodyKinematics(lastKinematics));
            assertEquals(lastKinematics, kinematics);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avgFx = avgFx * (double) i / (double) j + fxi / j;
            avgFy = avgFy * (double) i / (double) j + fyi / j;
            avgFz = avgFz * (double) i / (double) j + fzi / j;

            avgWx = avgWx * (double) i / (double) j + wxi / j;
            avgWy = avgWy * (double) i / (double) j + wyi / j;
            avgWz = avgWz * (double) i / (double) j + wzi / j;

            double diff = fxi - avgFx;
            double diff2 = diff * diff;
            varFx = varFx * (double) i / (double) j + diff2 / j;

            diff = fyi - avgFy;
            diff2 = diff * diff;
            varFy = varFy * (double) i / (double) j + diff2 / j;

            diff = fzi - avgFz;
            diff2 = diff * diff;
            varFz = varFz * (double) i / (double) j + diff2 / j;

            diff = wxi - avgWx;
            diff2 = diff * diff;
            varWx = varWx * (double) i / (double) j + diff2 / j;

            diff = wyi - avgWy;
            diff2 = diff * diff;
            varWy = varWy * (double) i / (double) j + diff2 / j;

            diff = wzi - avgWz;
            diff2 = diff * diff;
            varWz = varWz * (double) i / (double) j + diff2 / j;
        }

        assertEquals(estimator.getNumberOfProcessedSamples(), N_SAMPLES);
        assertFalse(estimator.isRunning());
        assertEquals(mStart, 1);
        assertEquals(mBodyKinematicsAdded, N_SAMPLES);
        assertEquals(mReset, 0);

        final double avgFxB = estimator.getAvgSpecificForceX();
        final double avgFyB = estimator.getAvgSpecificForceY();
        final double avgFzB = estimator.getAvgSpecificForceZ();

        final double avgWxB = estimator.getAvgAngularRateX();
        final double avgWyB = estimator.getAvgAngularRateY();
        final double avgWzB = estimator.getAvgAngularRateZ();

        assertEquals(avgFx, avgFxB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFy, avgFyB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFz, avgFzB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWx, avgWxB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWy, avgWyB, SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWz, avgWzB, SMALL_ABSOLUTE_ERROR);

        assertEquals(avgFxB, fx, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgFyB, fy, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgFzB, fz, LARGE_ABSOLUTE_ERROR);

        assertEquals(avgWxB, omegaX, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgWyB, omegaY, LARGE_ABSOLUTE_ERROR);
        assertEquals(avgWzB, omegaZ, LARGE_ABSOLUTE_ERROR);

        final Acceleration avgFx1 = estimator.getAvgSpecificForceXAsMeasurement();
        final Acceleration avgFx2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceXAsMeasurement(avgFx2);

        assertEquals(avgFx, avgFx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFx1.getUnit());
        assertEquals(avgFx1, avgFx2);

        final Acceleration avgFy1 = estimator.getAvgSpecificForceYAsMeasurement();
        final Acceleration avgFy2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceYAsMeasurement(avgFy2);

        assertEquals(avgFy, avgFy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFy1.getUnit());
        assertEquals(avgFy1, avgFy2);

        final Acceleration avgFz1 = estimator.getAvgSpecificForceZAsMeasurement();
        final Acceleration avgFz2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceZAsMeasurement(avgFz2);

        assertEquals(avgFz, avgFz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFz1.getUnit());
        assertEquals(avgFz1, avgFz2);

        final AccelerationTriad avgFTriad1 = estimator.getAvgSpecificForceAsTriad();
        assertEquals(avgFx, avgFTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFy, avgFTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFz, avgFTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFTriad1.getUnit());
        final AccelerationTriad avgFTriad2 = new AccelerationTriad();
        estimator.getAvgSpecificForceAsTriad(avgFTriad2);
        assertEquals(avgFTriad1, avgFTriad2);
        assertEquals(avgFTriad1.getNorm(), estimator.getAvgSpecificForceNorm(),
                0.0);
        final Acceleration avgFNorm1 = estimator.getAvgSpecificForceNormAsMeasurement();
        assertEquals(estimator.getAvgSpecificForceNorm(),
                avgFNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgFNorm1.getUnit());
        final Acceleration avgFNorm2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgSpecificForceNormAsMeasurement(avgFNorm2);
        assertEquals(avgFNorm1, avgFNorm2);

        final AngularSpeed avgWx1 = estimator.getAvgAngularRateXAsMeasurement();
        assertEquals(avgWx, avgWx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWx1.getUnit());
        final AngularSpeed avgWx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateXAsMeasurement(avgWx2);
        assertEquals(avgWx1, avgWx2);

        final AngularSpeed avgWy1 = estimator.getAvgAngularRateYAsMeasurement();
        assertEquals(avgWy, avgWy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWy1.getUnit());
        final AngularSpeed avgWy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateYAsMeasurement(avgWy2);
        assertEquals(avgWy1, avgWy2);

        final AngularSpeed avgWz1 = estimator.getAvgAngularRateZAsMeasurement();
        assertEquals(avgWz, avgWz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWz1.getUnit());
        final AngularSpeed avgWz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgAngularRateZAsMeasurement(avgWz2);
        assertEquals(avgWz1, avgWz2);

        final AngularSpeedTriad avgWTriad1 = estimator.getAvgAngularRateTriad();
        assertEquals(avgWx, avgWTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWy, avgWTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWz, avgWTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWTriad1.getUnit());
        final AngularSpeedTriad avgWTriad2 = new AngularSpeedTriad();
        estimator.getAvgAngularRateTriad(avgWTriad2);
        assertEquals(avgWTriad1, avgWTriad2);

        assertEquals(avgWTriad1.getNorm(), estimator.getAvgAngularRateNorm(),
                0.0);
        final AngularSpeed avgWNorm1 = estimator.getAvgAngularRateNormAsMeasurement();
        assertEquals(estimator.getAvgAngularRateNorm(),
                avgWNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgWNorm1.getUnit());
        final AngularSpeed avgWNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAvgAngularRateNormAsMeasurement(avgWNorm2);
        assertEquals(avgWNorm1, avgWNorm2);

        final BodyKinematics avgKinematics1 = estimator.getAvgBodyKinematics();
        assertEquals(avgFx, avgKinematics1.getFx(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFy, avgKinematics1.getFy(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgFz, avgKinematics1.getFz(), SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWx, avgKinematics1.getAngularRateX(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWy, avgKinematics1.getAngularRateY(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(avgWz, avgKinematics1.getAngularRateZ(),
                SMALL_ABSOLUTE_ERROR);
        final BodyKinematics avgKinematics2 = new BodyKinematics();
        estimator.getAvgBodyKinematics(avgKinematics2);
        assertEquals(avgKinematics1, avgKinematics2);

        assertEquals(varFx, estimator.getVarianceSpecificForceX(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(varFy, estimator.getVarianceSpecificForceY(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(varFz, estimator.getVarianceSpecificForceZ(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(varWx, estimator.getVarianceAngularRateX(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(varWy, estimator.getVarianceAngularRateY(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(varWz, estimator.getVarianceAngularRateZ(),
                SMALL_ABSOLUTE_ERROR);

        final double stdFx = Math.sqrt(varFx);
        final double stdFy = Math.sqrt(varFy);
        final double stdFz = Math.sqrt(varFz);
        final double stdWx = Math.sqrt(varWx);
        final double stdWy = Math.sqrt(varWy);
        final double stdWz = Math.sqrt(varWz);

        assertEquals(stdFx, estimator.getStandardDeviationSpecificForceX(),
                SMALL_ABSOLUTE_ERROR);
        final Acceleration stdFx1 = estimator.getStandardDeviationSpecificForceXAsMeasurement();
        assertEquals(stdFx, stdFx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                stdFx1.getUnit());
        final Acceleration stdFx2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceXAsMeasurement(stdFx2);
        assertEquals(stdFx1, stdFx2);

        assertEquals(stdFy, estimator.getStandardDeviationSpecificForceY(),
                SMALL_ABSOLUTE_ERROR);
        final Acceleration stdFy1 = estimator.getStandardDeviationSpecificForceYAsMeasurement();
        assertEquals(stdFy, stdFy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                stdFy1.getUnit());
        final Acceleration stdFy2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceYAsMeasurement(stdFy2);
        assertEquals(stdFy1, stdFy2);

        assertEquals(stdFz, estimator.getStandardDeviationSpecificForceZ(),
                SMALL_ABSOLUTE_ERROR);
        final Acceleration stdFz1 = estimator.getStandardDeviationSpecificForceZAsMeasurement();
        assertEquals(stdFz, stdFz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                stdFz1.getUnit());
        final Acceleration stdFz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceZAsMeasurement(stdFz2);
        assertEquals(stdFz1, stdFz2);

        final AccelerationTriad stdFTriad1 = estimator.getStandardDeviationSpecificForceTriad();
        assertEquals(stdFx, stdFTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFy, stdFTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFz, stdFTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                stdFTriad1.getUnit());

        final AccelerationTriad stdFTriad2 = new AccelerationTriad();
        estimator.getStandardDeviationSpecificForceTriad(stdFTriad2);
        assertEquals(stdFTriad1, stdFTriad2);

        assertEquals(stdFTriad1.getNorm(),
                estimator.getStandardDeviationSpecificForceNorm(),
                SMALL_ABSOLUTE_ERROR);
        final Acceleration stdFNorm1 = estimator.getStandardDeviationSpecificForceNormAsMeasurement();
        assertEquals(stdFTriad1.getNorm(), stdFNorm1.getValue().doubleValue(),
                0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                stdFNorm1.getUnit());
        final Acceleration stdFNorm2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationSpecificForceNormAsMeasurement(stdFNorm2);
        assertEquals(stdFNorm1, stdFNorm2);

        final double avgStdF = (stdFx + stdFy + stdFz) / 3.0;
        assertEquals(avgStdF, estimator.getAverageStandardDeviationSpecificForce(),
                SMALL_ABSOLUTE_ERROR);
        final Acceleration avgStdF1 = estimator.getAverageStandardDeviationSpecificForceAsMeasurement();
        assertEquals(avgStdF, avgStdF1.getValue().doubleValue(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avgStdF1.getUnit());
        final Acceleration avgStdF2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAverageStandardDeviationSpecificForceAsMeasurement(avgStdF2);
        assertEquals(avgStdF1, avgStdF2);

        assertEquals(stdWx, estimator.getStandardDeviationAngularRateX(),
                SMALL_ABSOLUTE_ERROR);
        final AngularSpeed stdWx1 = estimator.getStandardDeviationAngularRateXAsMeasurement();
        assertEquals(stdWx, stdWx1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWx1.getUnit());
        final AngularSpeed stdWx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateXAsMeasurement(stdWx2);
        assertEquals(stdWx1, stdWx2);

        assertEquals(stdWy, estimator.getStandardDeviationAngularRateY(),
                SMALL_ABSOLUTE_ERROR);
        final AngularSpeed stdWy1 = estimator.getStandardDeviationAngularRateYAsMeasurement();
        assertEquals(stdWy, stdWy1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWy1.getUnit());
        final AngularSpeed stdWy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateYAsMeasurement(stdWy2);
        assertEquals(stdWy1, stdWy2);

        assertEquals(stdWz, estimator.getStandardDeviationAngularRateZ(),
                SMALL_ABSOLUTE_ERROR);
        final AngularSpeed stdWz1 = estimator.getStandardDeviationAngularRateZAsMeasurement();
        assertEquals(stdWz, stdWz1.getValue().doubleValue(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWz1.getUnit());
        final AngularSpeed stdWz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularRateZAsMeasurement(stdWz2);
        assertEquals(stdWz1, stdWz2);

        final AngularSpeedTriad stdWTriad1 = estimator.getStandardDeviationAngularSpeedTriad();
        assertEquals(stdWx, stdWTriad1.getValueX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWy, stdWTriad1.getValueY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWz, stdWTriad1.getValueZ(), SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWTriad1.getUnit());
        final AngularSpeedTriad stdWTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationAngularSpeedTriad(stdWTriad2);
        assertEquals(stdWTriad1, stdWTriad2);

        assertEquals(stdWTriad1.getNorm(), estimator.getStandardDeviationAngularSpeedNorm(),
                0.0);
        final AngularSpeed stdWNorm1 = estimator.getStandardDeviationAngularSpeedNormAsMeasurement();
        assertEquals(stdWTriad1.getNorm(), stdWNorm1.getValue().doubleValue(),
                0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdWNorm1.getUnit());
        final AngularSpeed stdWNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationAngularSpeedNormAsMeasurement(stdWNorm2);
        assertEquals(stdWNorm1, stdWNorm2);

        final double avgStdW = (stdWx + stdWy + stdWz) / 3.0;
        assertEquals(avgStdW, estimator.getAverageStandardDeviationAngularSpeed(),
                SMALL_ABSOLUTE_ERROR);
        final AngularSpeed avgStdW1 = estimator.getAverageStandardDeviationAngularSpeedAsMeasurement();
        assertEquals(avgStdW, avgStdW1.getValue().doubleValue(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgStdW1.getUnit());
        final AngularSpeed avgStdW2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAverageStandardDeviationAngularSpeedAsMeasurement(avgStdW2);
        assertEquals(avgStdW1, avgStdW2);

        final BodyKinematics stdKinematics1 = estimator.getStandardDeviationAsBodyKinematics();
        assertEquals(stdFx, stdKinematics1.getFx(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFy, stdKinematics1.getFy(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdFz, stdKinematics1.getFz(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWx, stdKinematics1.getAngularRateX(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWy, stdKinematics1.getAngularRateY(), SMALL_ABSOLUTE_ERROR);
        assertEquals(stdWz, stdKinematics1.getAngularRateZ(), SMALL_ABSOLUTE_ERROR);
        final BodyKinematics stdKinematics2 = new BodyKinematics();
        estimator.getStandardDeviationAsBodyKinematics(stdKinematics2);
        assertEquals(stdKinematics1, stdKinematics2);

        final double psdFx = estimator.getSpecificForcePsdX();
        final double psdFy = estimator.getSpecificForcePsdY();
        final double psdFz = estimator.getSpecificForcePsdZ();
        final double psdWx = estimator.getAngularRatePsdX();
        final double psdWy = estimator.getAngularRatePsdY();
        final double psdWz = estimator.getAngularRatePsdZ();

        assertEquals(psdFx, varFx * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdFy, varFy * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdFz, varFz * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdWx, varWx * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdWy, varWy * timeInterval, SMALL_ABSOLUTE_ERROR);
        assertEquals(psdWz, varWz * timeInterval, SMALL_ABSOLUTE_ERROR);

        final double rootPsdFx = Math.sqrt(psdFx);
        final double rootPsdFy = Math.sqrt(psdFy);
        final double rootPsdFz = Math.sqrt(psdFz);
        final double rootPsdWx = Math.sqrt(psdWx);
        final double rootPsdWy = Math.sqrt(psdWy);
        final double rootPsdWz = Math.sqrt(psdWz);

        assertEquals(rootPsdFx, estimator.getSpecificForceRootPsdX(),
                0.0);
        assertEquals(rootPsdFy, estimator.getSpecificForceRootPsdY(),
                0.0);
        assertEquals(rootPsdFz, estimator.getSpecificForceRootPsdZ(),
                0.0);
        assertEquals(rootPsdWx, estimator.getAngularRateRootPsdX(),
                0.0);
        assertEquals(rootPsdWy, estimator.getAngularRateRootPsdY(),
                0.0);
        assertEquals(rootPsdWz, estimator.getAngularRateRootPsdZ(),
                0.0);

        final double avgPsdF = (psdFx + psdFy + psdFz) / 3.0;
        final double avgPsdW = (psdWx + psdWy + psdWz) / 3.0;
        final double normRootPsdF = Math.sqrt(
                rootPsdFx * rootPsdFx + rootPsdFy * rootPsdFy +
                        rootPsdFz * rootPsdFz);
        final double normRootPsdW = Math.sqrt(
                rootPsdWx * rootPsdWx + rootPsdWy * rootPsdWy +
                        rootPsdWz * rootPsdWz);

        assertEquals(avgPsdF, estimator.getAvgSpecificForceNoisePsd(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(normRootPsdF, estimator.getSpecificForceNoiseRootPsdNorm(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(avgPsdW, estimator.getAvgAngularRateNoisePsd(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(normRootPsdW, estimator.getAngularRateNoiseRootPsdNorm(),
                SMALL_ABSOLUTE_ERROR);

        assertEquals(rootPsdFx, accelNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdFy, accelNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdFz, accelNoiseRootPSD, ABSOLUTE_ERROR);

        assertEquals(rootPsdWx, gyroNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdWy, gyroNoiseRootPSD, ABSOLUTE_ERROR);
        assertEquals(rootPsdWz, gyroNoiseRootPSD, ABSOLUTE_ERROR);

        final double accelNoisePsd = accelNoiseRootPSD * accelNoiseRootPSD;
        assertEquals(estimator.getAvgSpecificForceNoisePsd(),
                accelNoisePsd, ABSOLUTE_ERROR);

        final double gyroNoisePsd = gyroNoiseRootPSD * gyroNoiseRootPSD;
        assertEquals(estimator.getAvgAngularRateNoisePsd(),
                gyroNoisePsd, ABSOLUTE_ERROR);

        // reset
        estimator.reset();

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getLastBodyKinematics());
        assertFalse(estimator.getLastBodyKinematics(null));
        assertFalse(estimator.isRunning());
        assertEquals(mReset, 1);
    }

    @Override
    public void onStart(final AccumulatedBodyKinematicsNoiseEstimator estimator) {
        checkLocked(estimator);
        mStart++;
    }

    @Override
    public void onBodyKinematicsAdded(final AccumulatedBodyKinematicsNoiseEstimator estimator) {
        mBodyKinematicsAdded++;
    }

    @Override
    public void onReset(final AccumulatedBodyKinematicsNoiseEstimator estimator) {
        mReset++;
    }

    private void reset() {
        mStart = 0;
        mBodyKinematicsAdded = 0;
        mReset = 0;
    }

    private void checkLocked(final AccumulatedBodyKinematicsNoiseEstimator estimator) {
        assertTrue(estimator.isRunning());
        try {
            estimator.setTimeInterval(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setTimeInterval(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.addBodyKinematics(0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        final Acceleration a = new Acceleration(
                0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AngularSpeed w = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        try {
            estimator.addBodyKinematics(a, a, a, w, w, w);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        final AccelerationTriad aTriad = new AccelerationTriad();
        final AngularSpeedTriad wTriad = new AngularSpeedTriad();
        try {
            estimator.addBodyKinematics(aTriad, wTriad);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        final BodyKinematics kinematics = new BodyKinematics();
        try {
            estimator.addBodyKinematics(kinematics);
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
