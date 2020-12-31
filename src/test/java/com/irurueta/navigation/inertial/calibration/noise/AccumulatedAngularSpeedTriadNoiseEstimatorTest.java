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
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class AccumulatedAngularSpeedTriadNoiseEstimatorTest implements
        AccumulatedAngularSpeedTriadNoiseEstimatorListener {

    private static final double MIN_GYRO_VALUE = -2.0;
    private static final double MAX_GYRO_VALUE = 2.0;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int N_SAMPLES = 1000;

    private int mStart;
    private int mTriadAdded;
    private int mReset;

    @Test
    public void testConstructor1() {
        final AccumulatedAngularSpeedTriadNoiseEstimator estimator =
                new AccumulatedAngularSpeedTriadNoiseEstimator();

        // check default values
        assertEquals(AccumulatedAngularSpeedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedAngularSpeedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
        assertEquals(0.0, estimator.getAvgX(), 0.0);
        final AngularSpeed avgX1 = estimator.getAvgXAsMeasurement();
        assertEquals(0.0, avgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgX1.getUnit());
        final AngularSpeed avgX2 = new AngularSpeed(
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAvgXAsMeasurement(avgX2);
        assertEquals(avgX1, avgX2);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        final AngularSpeed avgY1 = estimator.getAvgYAsMeasurement();
        assertEquals(0.0, avgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgY1.getUnit());
        final AngularSpeed avgY2 = new AngularSpeed(
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAvgYAsMeasurement(avgY2);
        assertEquals(avgY1, avgY2);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        final AngularSpeed avgZ1 = estimator.getAvgZAsMeasurement();
        assertEquals(0.0, avgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgZ1.getUnit());
        final AngularSpeed avgZ2 = new AngularSpeed(
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAvgZAsMeasurement(avgZ2);
        assertEquals(avgZ1, avgZ2);
        final AngularSpeedTriad triad1 = estimator.getAvgTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());
        final AngularSpeedTriad triad2 = new AngularSpeedTriad();
        estimator.getAvgTriad(triad2);
        assertEquals(triad1, triad2);
        assertEquals(0.0, estimator.getAvgNorm(), 0.0);
        final AngularSpeed norm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(0.0, norm1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, norm1.getUnit());
        final AngularSpeed norm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgNormAsMeasurement(norm2);
        assertEquals(norm1, norm2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final AngularSpeed stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdX1.getUnit());
        final AngularSpeed stdX2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final AngularSpeed stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdY1.getUnit());
        final AngularSpeed stdY2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final AngularSpeed stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdZ1.getUnit());
        final AngularSpeed stdZ2 = new AngularSpeed(
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final AngularSpeedTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdTriad1.getUnit());
        final AngularSpeedTriad stdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getStandardDeviationNorm(), 0.0);
        final AngularSpeed stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(0.0, stdNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdNorm1.getUnit());
        final AngularSpeed stdNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final AngularSpeed avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgStd1.getUnit());
        final AngularSpeed avgStd2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getNoiseRootPsdNorm(), 0.0);
        assertEquals(0.0, estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Test
    public void testConstructor2() {
        final AccumulatedAngularSpeedTriadNoiseEstimator estimator =
                new AccumulatedAngularSpeedTriadNoiseEstimator(this);

        // check default values
        assertEquals(AccumulatedAngularSpeedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedAngularSpeedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
        assertEquals(0.0, estimator.getAvgX(), 0.0);
        final AngularSpeed avgX1 = estimator.getAvgXAsMeasurement();
        assertEquals(0.0, avgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgX1.getUnit());
        final AngularSpeed avgX2 = new AngularSpeed(
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAvgXAsMeasurement(avgX2);
        assertEquals(avgX1, avgX2);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        final AngularSpeed avgY1 = estimator.getAvgYAsMeasurement();
        assertEquals(0.0, avgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgY1.getUnit());
        final AngularSpeed avgY2 = new AngularSpeed(
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAvgYAsMeasurement(avgY2);
        assertEquals(avgY1, avgY2);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        final AngularSpeed avgZ1 = estimator.getAvgZAsMeasurement();
        assertEquals(0.0, avgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgZ1.getUnit());
        final AngularSpeed avgZ2 = new AngularSpeed(
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAvgZAsMeasurement(avgZ2);
        assertEquals(avgZ1, avgZ2);
        final AngularSpeedTriad triad1 = estimator.getAvgTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());
        final AngularSpeedTriad triad2 = new AngularSpeedTriad();
        estimator.getAvgTriad(triad2);
        assertEquals(triad1, triad2);
        assertEquals(0.0, estimator.getAvgNorm(), 0.0);
        final AngularSpeed norm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(0.0, norm1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, norm1.getUnit());
        final AngularSpeed norm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgNormAsMeasurement(norm2);
        assertEquals(norm1, norm2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final AngularSpeed stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdX1.getUnit());
        final AngularSpeed stdX2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final AngularSpeed stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdY1.getUnit());
        final AngularSpeed stdY2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final AngularSpeed stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdZ1.getUnit());
        final AngularSpeed stdZ2 = new AngularSpeed(
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final AngularSpeedTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdTriad1.getUnit());
        final AngularSpeedTriad stdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getStandardDeviationNorm(), 0.0);
        final AngularSpeed stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(0.0, stdNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdNorm1.getUnit());
        final AngularSpeed stdNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final AngularSpeed avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgStd1.getUnit());
        final AngularSpeed avgStd2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getNoiseRootPsdNorm(), 0.0);
        assertEquals(0.0, estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Test
    public void testGetSetTimeInterval() throws LockedException {
        final AccumulatedAngularSpeedTriadNoiseEstimator estimator =
                new AccumulatedAngularSpeedTriadNoiseEstimator();

        // check default value
        assertEquals(AccumulatedAngularSpeedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final AccumulatedAngularSpeedTriadNoiseEstimator estimator =
                new AccumulatedAngularSpeedTriadNoiseEstimator();

        // check default value
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedAngularSpeedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final AccumulatedAngularSpeedTriadNoiseEstimator estimator =
                new AccumulatedAngularSpeedTriadNoiseEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testAddTriadAndReset1() throws LockedException, WrongSizeException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = getAccelNoiseRootPsd();
        final double gyroNoiseRootPSD = getGyroNoiseRootPsd();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);
        final double fx = 0.0;
        final double fy = 0.0;
        final double fz = 0.0;
        final double omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final BodyKinematics trueKinematics = new BodyKinematics(fx, fy, fz,
                omegaX, omegaY, omegaZ);

        final AccumulatedAngularSpeedTriadNoiseEstimator estimator =
                new AccumulatedAngularSpeedTriadNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mTriadAdded, 0);
        assertEquals(mReset, 0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
        assertFalse(estimator.isRunning());

        final AngularSpeedTriad triad = new AngularSpeedTriad();
        final BodyKinematics kinematics = new BodyKinematics();
        final double timeInterval = estimator.getTimeInterval();
        final AngularSpeedTriad lastTriad = new AngularSpeedTriad();
        double valueX;
        double valueY;
        double valueZ;
        double avgX = 0.0;
        double avgY = 0.0;
        double avgZ = 0.0;
        double varX = 0.0;
        double varY = 0.0;
        double varZ = 0.0;
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastTriad(lastTriad)) {
                assertEquals(estimator.getLastTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                    errors, random, kinematics);
            kinematics.getAngularRateTriad(triad);

            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            estimator.addTriad(valueX, valueY, valueZ);

            assertTrue(estimator.getLastTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avgX = avgX * (double) i / (double) j + valueX / j;
            avgY = avgY * (double) i / (double) j + valueY / j;
            avgZ = avgZ * (double) i / (double) j + valueZ / j;

            final double diffX = valueX - avgX;
            final double diffY = valueY - avgY;
            final double diffZ = valueZ - avgZ;

            final double diffX2 = diffX * diffX;
            final double diffY2 = diffY * diffY;
            final double diffZ2 = diffZ * diffZ;

            varX = varX * (double) i / (double) j + diffX2 / j;
            varY = varY * (double) i / (double) j + diffY2 / j;
            varZ = varZ * (double) i / (double) j + diffZ2 / j;
        }

        assertEquals(estimator.getNumberOfProcessedSamples(), N_SAMPLES);
        assertFalse(estimator.isRunning());
        assertEquals(mStart, 1);
        assertEquals(mTriadAdded, N_SAMPLES);
        assertEquals(mReset, 0);

        assertEquals(avgX, estimator.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(avgY, estimator.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(avgZ, estimator.getAvgZ(), ABSOLUTE_ERROR);

        final AngularSpeed avgX1 = estimator.getAvgXAsMeasurement();
        assertEquals(avgX, avgX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgX1.getUnit());
        final AngularSpeed avgX2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgXAsMeasurement(avgX2);
        assertEquals(avgX1, avgX2);

        final AngularSpeed avgY1 = estimator.getAvgYAsMeasurement();
        assertEquals(avgY, avgY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgY1.getUnit());
        final AngularSpeed avgY2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgYAsMeasurement(avgY2);
        assertEquals(avgY1, avgY2);

        final AngularSpeed avgZ1 = estimator.getAvgZAsMeasurement();
        assertEquals(avgZ, avgZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgZ1.getUnit());
        final AngularSpeed avgZ2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgZAsMeasurement(avgZ2);
        assertEquals(avgZ1, avgZ2);

        final AngularSpeedTriad avgTriad1 = estimator.getAvgTriad();
        assertEquals(avgX, avgTriad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(avgY, avgTriad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(avgZ, avgTriad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgTriad1.getUnit());
        final AngularSpeedTriad avgTriad2 = new AngularSpeedTriad();
        estimator.getAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        final double avgNorm = Math.sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ);
        assertEquals(avgNorm, estimator.getAvgNorm(), ABSOLUTE_ERROR);

        final AngularSpeed avgNorm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(avgNorm, avgNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgNorm1.getUnit());
        final AngularSpeed avgNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAvgNormAsMeasurement(avgNorm2);
        assertEquals(avgNorm1, avgNorm2);

        assertEquals(varX, estimator.getVarianceX(), ABSOLUTE_ERROR);
        assertEquals(varY, estimator.getVarianceY(), ABSOLUTE_ERROR);
        assertEquals(varZ, estimator.getVarianceZ(), ABSOLUTE_ERROR);

        final double stdX = Math.sqrt(varX);
        final double stdY = Math.sqrt(varY);
        final double stdZ = Math.sqrt(varZ);

        assertEquals(stdX, estimator.getStandardDeviationX(), ABSOLUTE_ERROR);
        assertEquals(stdY, estimator.getStandardDeviationY(), ABSOLUTE_ERROR);
        assertEquals(stdZ, estimator.getStandardDeviationZ(), ABSOLUTE_ERROR);

        final AngularSpeed stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(stdX, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdX1.getUnit());
        final AngularSpeed stdX2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);

        final AngularSpeed stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(stdY, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdY1.getUnit());
        final AngularSpeed stdY2 = new AngularSpeed(
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);

        final AngularSpeed stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(stdZ, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdZ1.getUnit());
        final AngularSpeed stdZ2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        final AngularSpeedTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(stdX, stdTriad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(stdY, stdTriad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(stdZ, stdTriad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdTriad1.getUnit());
        final AngularSpeedTriad stdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);

        final double stdNorm = Math.sqrt(stdX * stdX + stdY * stdY + stdZ * stdZ);
        assertEquals(stdNorm, estimator.getStandardDeviationNorm(), ABSOLUTE_ERROR);

        final AngularSpeed stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(stdNorm, stdNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdNorm1.getUnit());
        final AngularSpeed stdNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);

        final double avgStd = (stdX + stdY + stdZ) / 3.0;
        assertEquals(avgStd, estimator.getAverageStandardDeviation(), ABSOLUTE_ERROR);

        final AngularSpeed avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(avgStd, avgStd1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgStd1.getUnit());
        final AngularSpeed avgStd2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);

        final double psdX = timeInterval * varX;
        final double psdY = timeInterval * varY;
        final double psdZ = timeInterval * varZ;

        assertEquals(psdX, estimator.getPsdX(), ABSOLUTE_ERROR);
        assertEquals(psdY, estimator.getPsdY(), ABSOLUTE_ERROR);
        assertEquals(psdZ, estimator.getPsdZ(), ABSOLUTE_ERROR);

        final double rootPsdX = Math.sqrt(psdX);
        final double rootPsdY = Math.sqrt(psdY);
        final double rootPsdZ = Math.sqrt(psdZ);

        assertEquals(rootPsdX, estimator.getRootPsdX(), ABSOLUTE_ERROR);
        assertEquals(rootPsdY, estimator.getRootPsdY(), ABSOLUTE_ERROR);
        assertEquals(rootPsdZ, estimator.getRootPsdZ(), ABSOLUTE_ERROR);

        final double avgPsd = (psdX + psdY + psdZ) / 3.0;

        assertEquals(avgPsd, estimator.getAvgNoisePsd(), ABSOLUTE_ERROR);

        final double rootPsdNorm = Math.sqrt(rootPsdX * rootPsdX + rootPsdY * rootPsdY
                + rootPsdZ * rootPsdZ);

        assertEquals(rootPsdNorm, estimator.getNoiseRootPsdNorm(), ABSOLUTE_ERROR);
        assertEquals(estimator.getNoiseRootPsdNorm(),
                estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);

        // reset
        assertTrue(estimator.reset());

        assertEquals(mReset, 1);

        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
        assertEquals(0.0, estimator.getAvgX(), 0.0);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNorm(), 0.0);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getNoiseRootPsdNorm(), 0.0);
        assertEquals(0.0, estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Test
    public void testAddTriadAndReset2() throws LockedException, WrongSizeException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = getAccelNoiseRootPsd();
        final double gyroNoiseRootPSD = getGyroNoiseRootPsd();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);
        final double fx = 0.0;
        final double fy = 0.0;
        final double fz = 0.0;
        final double omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final BodyKinematics trueKinematics = new BodyKinematics(fx, fy, fz,
                omegaX, omegaY, omegaZ);

        final AccumulatedAngularSpeedTriadNoiseEstimator estimator =
                new AccumulatedAngularSpeedTriadNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mTriadAdded, 0);
        assertEquals(mReset, 0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
        assertFalse(estimator.isRunning());

        final AngularSpeedTriad triad = new AngularSpeedTriad();
        final BodyKinematics kinematics = new BodyKinematics();
        final double timeInterval = estimator.getTimeInterval();
        final AngularSpeedTriad lastTriad = new AngularSpeedTriad();
        double valueX;
        double valueY;
        double valueZ;
        double avgX = 0.0;
        double avgY = 0.0;
        double avgZ = 0.0;
        double varX = 0.0;
        double varY = 0.0;
        double varZ = 0.0;
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastTriad(lastTriad)) {
                assertEquals(estimator.getLastTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                    errors, random, kinematics);
            kinematics.getAngularRateTriad(triad);

            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            estimator.addTriad(triad);

            assertTrue(estimator.getLastTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avgX = avgX * (double) i / (double) j + valueX / j;
            avgY = avgY * (double) i / (double) j + valueY / j;
            avgZ = avgZ * (double) i / (double) j + valueZ / j;

            final double diffX = valueX - avgX;
            final double diffY = valueY - avgY;
            final double diffZ = valueZ - avgZ;

            final double diffX2 = diffX * diffX;
            final double diffY2 = diffY * diffY;
            final double diffZ2 = diffZ * diffZ;

            varX = varX * (double) i / (double) j + diffX2 / j;
            varY = varY * (double) i / (double) j + diffY2 / j;
            varZ = varZ * (double) i / (double) j + diffZ2 / j;
        }

        assertEquals(estimator.getNumberOfProcessedSamples(), N_SAMPLES);
        assertFalse(estimator.isRunning());
        assertEquals(mStart, 1);
        assertEquals(mTriadAdded, N_SAMPLES);
        assertEquals(mReset, 0);

        assertEquals(avgX, estimator.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(avgY, estimator.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(avgZ, estimator.getAvgZ(), ABSOLUTE_ERROR);

        final AngularSpeed avgX1 = estimator.getAvgXAsMeasurement();
        assertEquals(avgX, avgX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgX1.getUnit());
        final AngularSpeed avgX2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgXAsMeasurement(avgX2);
        assertEquals(avgX1, avgX2);

        final AngularSpeed avgY1 = estimator.getAvgYAsMeasurement();
        assertEquals(avgY, avgY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgY1.getUnit());
        final AngularSpeed avgY2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgYAsMeasurement(avgY2);
        assertEquals(avgY1, avgY2);

        final AngularSpeed avgZ1 = estimator.getAvgZAsMeasurement();
        assertEquals(avgZ, avgZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgZ1.getUnit());
        final AngularSpeed avgZ2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgZAsMeasurement(avgZ2);
        assertEquals(avgZ1, avgZ2);

        final AngularSpeedTriad avgTriad1 = estimator.getAvgTriad();
        assertEquals(avgX, avgTriad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(avgY, avgTriad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(avgZ, avgTriad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgTriad1.getUnit());
        final AngularSpeedTriad avgTriad2 = new AngularSpeedTriad();
        estimator.getAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        final double avgNorm = Math.sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ);
        assertEquals(avgNorm, estimator.getAvgNorm(), ABSOLUTE_ERROR);

        final AngularSpeed avgNorm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(avgNorm, avgNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgNorm1.getUnit());
        final AngularSpeed avgNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAvgNormAsMeasurement(avgNorm2);
        assertEquals(avgNorm1, avgNorm2);

        assertEquals(varX, estimator.getVarianceX(), ABSOLUTE_ERROR);
        assertEquals(varY, estimator.getVarianceY(), ABSOLUTE_ERROR);
        assertEquals(varZ, estimator.getVarianceZ(), ABSOLUTE_ERROR);

        final double stdX = Math.sqrt(varX);
        final double stdY = Math.sqrt(varY);
        final double stdZ = Math.sqrt(varZ);

        assertEquals(stdX, estimator.getStandardDeviationX(), ABSOLUTE_ERROR);
        assertEquals(stdY, estimator.getStandardDeviationY(), ABSOLUTE_ERROR);
        assertEquals(stdZ, estimator.getStandardDeviationZ(), ABSOLUTE_ERROR);

        final AngularSpeed stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(stdX, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdX1.getUnit());
        final AngularSpeed stdX2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);

        final AngularSpeed stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(stdY, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdY1.getUnit());
        final AngularSpeed stdY2 = new AngularSpeed(
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);

        final AngularSpeed stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(stdZ, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdZ1.getUnit());
        final AngularSpeed stdZ2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        final AngularSpeedTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(stdX, stdTriad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(stdY, stdTriad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(stdZ, stdTriad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdTriad1.getUnit());
        final AngularSpeedTriad stdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);

        final double stdNorm = Math.sqrt(stdX * stdX + stdY * stdY + stdZ * stdZ);
        assertEquals(stdNorm, estimator.getStandardDeviationNorm(), ABSOLUTE_ERROR);

        final AngularSpeed stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(stdNorm, stdNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdNorm1.getUnit());
        final AngularSpeed stdNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);

        final double avgStd = (stdX + stdY + stdZ) / 3.0;
        assertEquals(avgStd, estimator.getAverageStandardDeviation(), ABSOLUTE_ERROR);

        final AngularSpeed avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(avgStd, avgStd1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgStd1.getUnit());
        final AngularSpeed avgStd2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);

        final double psdX = timeInterval * varX;
        final double psdY = timeInterval * varY;
        final double psdZ = timeInterval * varZ;

        assertEquals(psdX, estimator.getPsdX(), ABSOLUTE_ERROR);
        assertEquals(psdY, estimator.getPsdY(), ABSOLUTE_ERROR);
        assertEquals(psdZ, estimator.getPsdZ(), ABSOLUTE_ERROR);

        final double rootPsdX = Math.sqrt(psdX);
        final double rootPsdY = Math.sqrt(psdY);
        final double rootPsdZ = Math.sqrt(psdZ);

        assertEquals(rootPsdX, estimator.getRootPsdX(), ABSOLUTE_ERROR);
        assertEquals(rootPsdY, estimator.getRootPsdY(), ABSOLUTE_ERROR);
        assertEquals(rootPsdZ, estimator.getRootPsdZ(), ABSOLUTE_ERROR);

        final double avgPsd = (psdX + psdY + psdZ) / 3.0;

        assertEquals(avgPsd, estimator.getAvgNoisePsd(), ABSOLUTE_ERROR);

        final double rootPsdNorm = Math.sqrt(rootPsdX * rootPsdX + rootPsdY * rootPsdY
                + rootPsdZ * rootPsdZ);

        assertEquals(rootPsdNorm, estimator.getNoiseRootPsdNorm(), ABSOLUTE_ERROR);
        assertEquals(estimator.getNoiseRootPsdNorm(),
                estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);

        // reset
        assertTrue(estimator.reset());

        assertEquals(mReset, 1);

        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
        assertEquals(0.0, estimator.getAvgX(), 0.0);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNorm(), 0.0);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getNoiseRootPsdNorm(), 0.0);
        assertEquals(0.0, estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Test
    public void testAddTriadAndReset3() throws LockedException, WrongSizeException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMa();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = getAccelNoiseRootPsd();
        final double gyroNoiseRootPSD = getGyroNoiseRootPsd();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);
        final double fx = 0.0;
        final double fy = 0.0;
        final double fz = 0.0;
        final double omegaX = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaY = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);
        final double omegaZ = randomizer.nextDouble(MIN_GYRO_VALUE, MAX_GYRO_VALUE);

        final BodyKinematics trueKinematics = new BodyKinematics(fx, fy, fz,
                omegaX, omegaY, omegaZ);

        final AccumulatedAngularSpeedTriadNoiseEstimator estimator =
                new AccumulatedAngularSpeedTriadNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mTriadAdded, 0);
        assertEquals(mReset, 0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
        assertFalse(estimator.isRunning());

        final AngularSpeedTriad triad = new AngularSpeedTriad();
        final BodyKinematics kinematics = new BodyKinematics();
        final double timeInterval = estimator.getTimeInterval();
        final AngularSpeedTriad lastTriad = new AngularSpeedTriad();
        double valueX;
        double valueY;
        double valueZ;
        double avgX = 0.0;
        double avgY = 0.0;
        double avgZ = 0.0;
        double varX = 0.0;
        double varY = 0.0;
        double varZ = 0.0;
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastTriad(lastTriad)) {
                assertEquals(estimator.getLastTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                    errors, random, kinematics);
            kinematics.getAngularRateTriad(triad);

            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            estimator.addTriad(
                    triad.getMeasurementX(), triad.getMeasurementY(), triad.getMeasurementZ());

            assertTrue(estimator.getLastTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avgX = avgX * (double) i / (double) j + valueX / j;
            avgY = avgY * (double) i / (double) j + valueY / j;
            avgZ = avgZ * (double) i / (double) j + valueZ / j;

            final double diffX = valueX - avgX;
            final double diffY = valueY - avgY;
            final double diffZ = valueZ - avgZ;

            final double diffX2 = diffX * diffX;
            final double diffY2 = diffY * diffY;
            final double diffZ2 = diffZ * diffZ;

            varX = varX * (double) i / (double) j + diffX2 / j;
            varY = varY * (double) i / (double) j + diffY2 / j;
            varZ = varZ * (double) i / (double) j + diffZ2 / j;
        }

        assertEquals(estimator.getNumberOfProcessedSamples(), N_SAMPLES);
        assertFalse(estimator.isRunning());
        assertEquals(mStart, 1);
        assertEquals(mTriadAdded, N_SAMPLES);
        assertEquals(mReset, 0);

        assertEquals(avgX, estimator.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(avgY, estimator.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(avgZ, estimator.getAvgZ(), ABSOLUTE_ERROR);

        final AngularSpeed avgX1 = estimator.getAvgXAsMeasurement();
        assertEquals(avgX, avgX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgX1.getUnit());
        final AngularSpeed avgX2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgXAsMeasurement(avgX2);
        assertEquals(avgX1, avgX2);

        final AngularSpeed avgY1 = estimator.getAvgYAsMeasurement();
        assertEquals(avgY, avgY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgY1.getUnit());
        final AngularSpeed avgY2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgYAsMeasurement(avgY2);
        assertEquals(avgY1, avgY2);

        final AngularSpeed avgZ1 = estimator.getAvgZAsMeasurement();
        assertEquals(avgZ, avgZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgZ1.getUnit());
        final AngularSpeed avgZ2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgZAsMeasurement(avgZ2);
        assertEquals(avgZ1, avgZ2);

        final AngularSpeedTriad avgTriad1 = estimator.getAvgTriad();
        assertEquals(avgX, avgTriad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(avgY, avgTriad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(avgZ, avgTriad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgTriad1.getUnit());
        final AngularSpeedTriad avgTriad2 = new AngularSpeedTriad();
        estimator.getAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        final double avgNorm = Math.sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ);
        assertEquals(avgNorm, estimator.getAvgNorm(), ABSOLUTE_ERROR);

        final AngularSpeed avgNorm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(avgNorm, avgNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgNorm1.getUnit());
        final AngularSpeed avgNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getAvgNormAsMeasurement(avgNorm2);
        assertEquals(avgNorm1, avgNorm2);

        assertEquals(varX, estimator.getVarianceX(), ABSOLUTE_ERROR);
        assertEquals(varY, estimator.getVarianceY(), ABSOLUTE_ERROR);
        assertEquals(varZ, estimator.getVarianceZ(), ABSOLUTE_ERROR);

        final double stdX = Math.sqrt(varX);
        final double stdY = Math.sqrt(varY);
        final double stdZ = Math.sqrt(varZ);

        assertEquals(stdX, estimator.getStandardDeviationX(), ABSOLUTE_ERROR);
        assertEquals(stdY, estimator.getStandardDeviationY(), ABSOLUTE_ERROR);
        assertEquals(stdZ, estimator.getStandardDeviationZ(), ABSOLUTE_ERROR);

        final AngularSpeed stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(stdX, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdX1.getUnit());
        final AngularSpeed stdX2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);

        final AngularSpeed stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(stdY, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdY1.getUnit());
        final AngularSpeed stdY2 = new AngularSpeed(
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);

        final AngularSpeed stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(stdZ, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdZ1.getUnit());
        final AngularSpeed stdZ2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        final AngularSpeedTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(stdX, stdTriad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(stdY, stdTriad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(stdZ, stdTriad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdTriad1.getUnit());
        final AngularSpeedTriad stdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);

        final double stdNorm = Math.sqrt(stdX * stdX + stdY * stdY + stdZ * stdZ);
        assertEquals(stdNorm, estimator.getStandardDeviationNorm(), ABSOLUTE_ERROR);

        final AngularSpeed stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(stdNorm, stdNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdNorm1.getUnit());
        final AngularSpeed stdNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);

        final double avgStd = (stdX + stdY + stdZ) / 3.0;
        assertEquals(avgStd, estimator.getAverageStandardDeviation(), ABSOLUTE_ERROR);

        final AngularSpeed avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(avgStd, avgStd1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgStd1.getUnit());
        final AngularSpeed avgStd2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);

        final double psdX = timeInterval * varX;
        final double psdY = timeInterval * varY;
        final double psdZ = timeInterval * varZ;

        assertEquals(psdX, estimator.getPsdX(), ABSOLUTE_ERROR);
        assertEquals(psdY, estimator.getPsdY(), ABSOLUTE_ERROR);
        assertEquals(psdZ, estimator.getPsdZ(), ABSOLUTE_ERROR);

        final double rootPsdX = Math.sqrt(psdX);
        final double rootPsdY = Math.sqrt(psdY);
        final double rootPsdZ = Math.sqrt(psdZ);

        assertEquals(rootPsdX, estimator.getRootPsdX(), ABSOLUTE_ERROR);
        assertEquals(rootPsdY, estimator.getRootPsdY(), ABSOLUTE_ERROR);
        assertEquals(rootPsdZ, estimator.getRootPsdZ(), ABSOLUTE_ERROR);

        final double avgPsd = (psdX + psdY + psdZ) / 3.0;

        assertEquals(avgPsd, estimator.getAvgNoisePsd(), ABSOLUTE_ERROR);

        final double rootPsdNorm = Math.sqrt(rootPsdX * rootPsdX + rootPsdY * rootPsdY
                + rootPsdZ * rootPsdZ);

        assertEquals(rootPsdNorm, estimator.getNoiseRootPsdNorm(), ABSOLUTE_ERROR);
        assertEquals(estimator.getNoiseRootPsdNorm(),
                estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);

        // reset
        assertTrue(estimator.reset());

        assertEquals(mReset, 1);

        assertNull(estimator.getLastTriad());
        assertFalse(estimator.getLastTriad(null));
        assertEquals(0.0, estimator.getAvgX(), 0.0);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNorm(), 0.0);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        assertEquals(0.0, estimator.getPsdX(), 0.0);
        assertEquals(0.0, estimator.getPsdY(), 0.0);
        assertEquals(0.0, estimator.getPsdZ(), 0.0);
        assertEquals(0.0, estimator.getRootPsdX(), 0.0);
        assertEquals(0.0, estimator.getRootPsdY(), 0.0);
        assertEquals(0.0, estimator.getRootPsdZ(), 0.0);
        assertEquals(0.0, estimator.getAvgNoisePsd(), 0.0);
        assertEquals(0.0, estimator.getNoiseRootPsdNorm(), 0.0);
        assertEquals(0.0, estimator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Override
    public void onStart(final AccumulatedAngularSpeedTriadNoiseEstimator estimator) {
        checkLocked(estimator);
        mStart++;
    }

    @Override
    public void onTriadAdded(final AccumulatedAngularSpeedTriadNoiseEstimator estimator) {
        mTriadAdded++;
    }

    @Override
    public void onReset(final AccumulatedAngularSpeedTriadNoiseEstimator estimator) {
        mReset++;
    }

    private void reset() {
        mStart = 0;
        mTriadAdded = 0;
        mReset = 0;
    }

    private void checkLocked(final AccumulatedAngularSpeedTriadNoiseEstimator estimator) {
        assertTrue(estimator.isRunning());
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
            estimator.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.addTriad(0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.addTriad(new AngularSpeedTriad());
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        final AngularSpeed w = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        try {
            estimator.addTriad(w, w, w);
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

    private double getAccelNoiseRootPsd() {
        return 100.0 * MICRO_G_TO_METERS_PER_SECOND_SQUARED;
    }

    private double getGyroNoiseRootPsd() {
        return 0.01 * DEG_TO_RAD / 60.0;
    }
}
