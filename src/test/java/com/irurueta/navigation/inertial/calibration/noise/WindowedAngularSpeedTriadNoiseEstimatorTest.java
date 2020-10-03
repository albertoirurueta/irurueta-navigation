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

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class WindowedAngularSpeedTriadNoiseEstimatorTest implements WindowedAngularSpeedTriadNoiseEstimatorListener {

    private static final double MIN_GYRO_VALUE = -2.0;
    private static final double MAX_GYRO_VALUE = 2.0;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private int mStart;
    private int mTriadAdded;
    private int mWindowFilled;
    private int mReset;

    @Test
    public void testConstructor1() {
        final WindowedAngularSpeedTriadNoiseEstimator estimator =
                new WindowedAngularSpeedTriadNoiseEstimator();

        // check default values
        assertEquals(WindowedAngularSpeedTriadNoiseEstimator.DEFAULT_WINDOW_SIZE,
                estimator.getWindowSize());
        assertEquals(WindowedAngularSpeedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedAngularSpeedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertNull(estimator.getListener());
        assertNull(estimator.getFirstWindowedTriad());
        assertFalse(estimator.getFirstWindowedTriad(null));
        assertNull(estimator.getLastWindowedTriad());
        assertFalse(estimator.getLastWindowedTriad(null));
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
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
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
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final AngularSpeed stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdY1.getUnit());
        final AngularSpeed stdY2 = new AngularSpeed(
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
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
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isWindowFilled());
    }

    @Test
    public void testConstructor2() {
        final WindowedAngularSpeedTriadNoiseEstimator estimator =
                new WindowedAngularSpeedTriadNoiseEstimator(this);

        // check default values
        assertEquals(WindowedAngularSpeedTriadNoiseEstimator.DEFAULT_WINDOW_SIZE,
                estimator.getWindowSize());
        assertEquals(WindowedAngularSpeedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedAngularSpeedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getFirstWindowedTriad());
        assertFalse(estimator.getFirstWindowedTriad(null));
        assertNull(estimator.getLastWindowedTriad());
        assertFalse(estimator.getLastWindowedTriad(null));
        assertEquals(0.0, estimator.getAvgX(), 0.0);
        final AngularSpeed avgX1 = estimator.getAvgXAsMeasurement();
        assertEquals(0.0, avgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgX1.getUnit());
        final AngularSpeed avgX2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgXAsMeasurement(avgX2);
        assertEquals(avgX1, avgX2);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        final AngularSpeed avgY1 = estimator.getAvgYAsMeasurement();
        assertEquals(0.0, avgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgY1.getUnit());
        final AngularSpeed avgY2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgYAsMeasurement(avgY2);
        assertEquals(avgY1, avgY2);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        final AngularSpeed avgZ1 = estimator.getAvgZAsMeasurement();
        assertEquals(0.0, avgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgZ1.getUnit());
        final AngularSpeed avgZ2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
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
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
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
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
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
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isWindowFilled());
    }

    @Test
    public void testGetSetWindowSize() throws LockedException {
        final WindowedAngularSpeedTriadNoiseEstimator estimator =
                new WindowedAngularSpeedTriadNoiseEstimator();

        // check default value
        assertEquals(WindowedAngularSpeedTriadNoiseEstimator.DEFAULT_WINDOW_SIZE,
                estimator.getWindowSize());

        // set new value
        estimator.setWindowSize(3);

        // check
        assertEquals(3, estimator.getWindowSize());

        // force IllegalArgumentException
        try {
            estimator.setWindowSize(1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetTimeInterval() throws LockedException {
        final WindowedAngularSpeedTriadNoiseEstimator estimator =
                new WindowedAngularSpeedTriadNoiseEstimator();

        // check default value
        assertEquals(WindowedAngularSpeedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final WindowedAngularSpeedTriadNoiseEstimator estimator =
                new WindowedAngularSpeedTriadNoiseEstimator();

        // check default value
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedAngularSpeedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final WindowedAngularSpeedTriadNoiseEstimator estimator =
                new WindowedAngularSpeedTriadNoiseEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testAddTriadAndProcessAndThenReset1() throws WrongSizeException, LockedException {
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

        final WindowedAngularSpeedTriadNoiseEstimator estimator =
                new WindowedAngularSpeedTriadNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mTriadAdded, 0);
        assertEquals(mWindowFilled, 0);
        assertEquals(mReset, 0);
        assertFalse(estimator.isWindowFilled());
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getFirstWindowedTriad());
        assertNull(estimator.getLastWindowedTriad());
        assertFalse(estimator.isRunning());

        final BodyKinematics kinematics = new BodyKinematics();
        final BodyKinematics firstKinematics = new BodyKinematics();
        final int windowSize = estimator.getWindowSize();
        final double timeInterval = estimator.getTimeInterval();
        final AngularSpeedTriad firstTriad = new AngularSpeedTriad();
        final AngularSpeedTriad lastTriad = new AngularSpeedTriad();
        final AngularSpeedTriad triad = new AngularSpeedTriad();
        double valueX;
        double valueY;
        double valueZ;
        double avgWx = 0.0;
        double avgWy = 0.0;
        double avgWz = 0.0;
        final List<AngularSpeedTriad> triads = new ArrayList<>();
        for (int i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedTriad(firstTriad)) {
                assertEquals(estimator.getFirstWindowedTriad(), firstTriad);
                assertEquals(firstTriad, firstKinematics.getAngularRateTriad());
            }
            if (estimator.getLastWindowedTriad(lastTriad)) {
                assertEquals(estimator.getLastWindowedTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                    errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getAngularRateTriad(triad);
            triads.add(new AngularSpeedTriad(triad));
            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            assertEquals(estimator.addTriadAndProcess(triad), i != 0);

            assertTrue(estimator.getLastWindowedTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avgWx += valueX;
            avgWy += valueY;
            avgWz += valueZ;
        }

        avgWx /= windowSize;
        avgWy /= windowSize;
        avgWz /= windowSize;

        double varWx = 0.0;
        double varWy = 0.0;
        double varWz = 0.0;
        for (int i = 0; i < windowSize; i++) {
            triad.copyFrom(triads.get(i));

            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            final double diffX = valueX - avgWx;
            final double diffY = valueY - avgWy;
            final double diffZ = valueZ - avgWz;

            varWx += diffX * diffX;
            varWy += diffY * diffY;
            varWz += diffZ * diffZ;
        }

        varWx /= (windowSize - 1);
        varWy /= (windowSize - 1);
        varWz /= (windowSize - 1);

        final double stdWx = Math.sqrt(varWx);
        final double stdWy = Math.sqrt(varWy);
        final double stdWz = Math.sqrt(varWz);

        assertEquals(avgWx, estimator.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(avgWy, estimator.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(avgWz, estimator.getAvgZ(), ABSOLUTE_ERROR);

        AngularSpeed w1 = estimator.getAvgXAsMeasurement();
        assertEquals(avgWx, w1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        final AngularSpeed w2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgXAsMeasurement(w2);
        assertEquals(w1, w2);

        w1 = estimator.getAvgYAsMeasurement();
        assertEquals(avgWy, w1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        estimator.getAvgYAsMeasurement(w2);
        assertEquals(w1, w2);

        w1 = estimator.getAvgZAsMeasurement();
        assertEquals(avgWz, w1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        estimator.getAvgZAsMeasurement(w2);
        assertEquals(w1, w2);

        final AngularSpeedTriad triad1 = estimator.getAvgTriad();
        assertEquals(avgWx, triad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(avgWy, triad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(avgWz, triad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());
        final AngularSpeedTriad triad2 = new AngularSpeedTriad();
        estimator.getAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(triad1.getNorm(), estimator.getAvgNorm(), ABSOLUTE_ERROR);
        final AngularSpeed norm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(triad1.getNorm(), norm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, norm1.getUnit());
        final AngularSpeed norm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgNormAsMeasurement(norm2);
        assertEquals(norm1, norm2);

        assertEquals(varWx, estimator.getVarianceX(), ABSOLUTE_ERROR);
        assertEquals(varWy, estimator.getVarianceY(), ABSOLUTE_ERROR);
        assertEquals(varWz, estimator.getVarianceZ(), ABSOLUTE_ERROR);

        assertEquals(stdWx, estimator.getStandardDeviationX(), ABSOLUTE_ERROR);
        assertEquals(stdWy, estimator.getStandardDeviationY(), ABSOLUTE_ERROR);
        assertEquals(stdWz, estimator.getStandardDeviationZ(), ABSOLUTE_ERROR);

        final AngularSpeed stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(stdWx, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdX1.getUnit());
        final AngularSpeed stdX2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);

        final AngularSpeed stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(stdWy, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdY1.getUnit());
        final AngularSpeed stdY2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);

        final AngularSpeed stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(stdWz, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdZ1.getUnit());
        final AngularSpeed stdZ2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        final AngularSpeedTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(stdWx, stdTriad1.getValueX(), 0.0);
        assertEquals(stdWy, stdTriad1.getValueY(), 0.0);
        assertEquals(stdWz, stdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdTriad1.getUnit());
        final AngularSpeedTriad stdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);

        assertEquals(stdTriad1.getNorm(), estimator.getStandardDeviationNorm(), ABSOLUTE_ERROR);
        final AngularSpeed stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(stdTriad1.getNorm(), stdNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdNorm1.getUnit());
        final AngularSpeed stdNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);

        final double avgStd = (stdWx + stdWy + stdWz) / 3.0;
        assertEquals(avgStd, estimator.getAverageStandardDeviation(), ABSOLUTE_ERROR);
        final AngularSpeed avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(avgStd, avgStd1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgStd1.getUnit());
        final AngularSpeed avgStd2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);

        final double psdX = varWx * timeInterval;
        final double psdY = varWy * timeInterval;
        final double psdZ = varWz * timeInterval;

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

        final double normRootPsd = Math.sqrt(rootPsdX * rootPsdX
                + rootPsdY * rootPsdY + rootPsdZ * rootPsdZ);
        assertEquals(normRootPsd, estimator.getNoiseRootPsdNorm(), ABSOLUTE_ERROR);

        assertEquals(windowSize, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());

        assertEquals(mStart, 1);
        assertEquals(mTriadAdded, windowSize);
        assertEquals(mWindowFilled, 1);
        assertEquals(mReset, 0);

        // if we add more triads, window filled is not called again
        BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                errors, random, kinematics);
        kinematics.getAngularRateTriad(triad);

        triads.add(new AngularSpeedTriad(triad));

        assertTrue(estimator.addTriadAndProcess(triad));

        assertEquals(windowSize + 1, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());

        assertEquals(triads.size(), windowSize + 1);
        assertEquals(triads.get(1), estimator.getFirstWindowedTriad());
        assertEquals(triads.get(windowSize), estimator.getLastWindowedTriad());

        assertEquals(mStart, 1);
        assertEquals(mTriadAdded, windowSize + 1);
        assertEquals(mWindowFilled, 1);
        assertEquals(mReset, 0);

        // reset
        assertTrue(estimator.reset());

        assertEquals(mReset, 1);

        assertNull(estimator.getFirstWindowedTriad());
        assertNull(estimator.getLastWindowedTriad());
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
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isWindowFilled());
    }

    @Test
    public void testAddTriadAndProcessAndThenReset2() throws WrongSizeException, LockedException {
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

        final WindowedAngularSpeedTriadNoiseEstimator estimator =
                new WindowedAngularSpeedTriadNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mTriadAdded, 0);
        assertEquals(mWindowFilled, 0);
        assertEquals(mReset, 0);
        assertFalse(estimator.isWindowFilled());
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getFirstWindowedTriad());
        assertNull(estimator.getLastWindowedTriad());
        assertFalse(estimator.isRunning());

        final BodyKinematics kinematics = new BodyKinematics();
        final BodyKinematics firstKinematics = new BodyKinematics();
        final int windowSize = estimator.getWindowSize();
        final double timeInterval = estimator.getTimeInterval();
        final AngularSpeedTriad firstTriad = new AngularSpeedTriad();
        final AngularSpeedTriad lastTriad = new AngularSpeedTriad();
        final AngularSpeedTriad triad = new AngularSpeedTriad();
        double valueX;
        double valueY;
        double valueZ;
        double avgWx = 0.0;
        double avgWy = 0.0;
        double avgWz = 0.0;
        final List<AngularSpeedTriad> triads = new ArrayList<>();
        for (int i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedTriad(firstTriad)) {
                assertEquals(estimator.getFirstWindowedTriad(), firstTriad);
                assertEquals(firstTriad, firstKinematics.getAngularRateTriad());
            }
            if (estimator.getLastWindowedTriad(lastTriad)) {
                assertEquals(estimator.getLastWindowedTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                    errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getAngularRateTriad(triad);
            triads.add(new AngularSpeedTriad(triad));
            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            assertEquals(estimator.addTriadAndProcess(valueX, valueY, valueZ), i != 0);

            assertTrue(estimator.getLastWindowedTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avgWx += valueX;
            avgWy += valueY;
            avgWz += valueZ;
        }

        avgWx /= windowSize;
        avgWy /= windowSize;
        avgWz /= windowSize;

        double varWx = 0.0;
        double varWy = 0.0;
        double varWz = 0.0;
        for (int i = 0; i < windowSize; i++) {
            triad.copyFrom(triads.get(i));

            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            final double diffX = valueX - avgWx;
            final double diffY = valueY - avgWy;
            final double diffZ = valueZ - avgWz;

            varWx += diffX * diffX;
            varWy += diffY * diffY;
            varWz += diffZ * diffZ;
        }

        varWx /= (windowSize - 1);
        varWy /= (windowSize - 1);
        varWz /= (windowSize - 1);

        final double stdWx = Math.sqrt(varWx);
        final double stdWy = Math.sqrt(varWy);
        final double stdWz = Math.sqrt(varWz);

        assertEquals(avgWx, estimator.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(avgWy, estimator.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(avgWz, estimator.getAvgZ(), ABSOLUTE_ERROR);

        AngularSpeed w1 = estimator.getAvgXAsMeasurement();
        assertEquals(avgWx, w1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        final AngularSpeed w2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgXAsMeasurement(w2);
        assertEquals(w1, w2);

        w1 = estimator.getAvgYAsMeasurement();
        assertEquals(avgWy, w1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        estimator.getAvgYAsMeasurement(w2);
        assertEquals(w1, w2);

        w1 = estimator.getAvgZAsMeasurement();
        assertEquals(avgWz, w1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        estimator.getAvgZAsMeasurement(w2);
        assertEquals(w1, w2);

        final AngularSpeedTriad triad1 = estimator.getAvgTriad();
        assertEquals(avgWx, triad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(avgWy, triad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(avgWz, triad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());
        final AngularSpeedTriad triad2 = new AngularSpeedTriad();
        estimator.getAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(triad1.getNorm(), estimator.getAvgNorm(), ABSOLUTE_ERROR);
        final AngularSpeed norm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(triad1.getNorm(), norm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, norm1.getUnit());
        final AngularSpeed norm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgNormAsMeasurement(norm2);
        assertEquals(norm1, norm2);

        assertEquals(varWx, estimator.getVarianceX(), ABSOLUTE_ERROR);
        assertEquals(varWy, estimator.getVarianceY(), ABSOLUTE_ERROR);
        assertEquals(varWz, estimator.getVarianceZ(), ABSOLUTE_ERROR);

        assertEquals(stdWx, estimator.getStandardDeviationX(), ABSOLUTE_ERROR);
        assertEquals(stdWy, estimator.getStandardDeviationY(), ABSOLUTE_ERROR);
        assertEquals(stdWz, estimator.getStandardDeviationZ(), ABSOLUTE_ERROR);

        final AngularSpeed stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(stdWx, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdX1.getUnit());
        final AngularSpeed stdX2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);

        final AngularSpeed stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(stdWy, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdY1.getUnit());
        final AngularSpeed stdY2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);

        final AngularSpeed stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(stdWz, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdZ1.getUnit());
        final AngularSpeed stdZ2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        final AngularSpeedTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(stdWx, stdTriad1.getValueX(), 0.0);
        assertEquals(stdWy, stdTriad1.getValueY(), 0.0);
        assertEquals(stdWz, stdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdTriad1.getUnit());
        final AngularSpeedTriad stdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);

        assertEquals(stdTriad1.getNorm(), estimator.getStandardDeviationNorm(), ABSOLUTE_ERROR);
        final AngularSpeed stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(stdTriad1.getNorm(), stdNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdNorm1.getUnit());
        final AngularSpeed stdNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);

        final double avgStd = (stdWx + stdWy + stdWz) / 3.0;
        assertEquals(avgStd, estimator.getAverageStandardDeviation(), ABSOLUTE_ERROR);
        final AngularSpeed avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(avgStd, avgStd1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgStd1.getUnit());
        final AngularSpeed avgStd2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);

        final double psdX = varWx * timeInterval;
        final double psdY = varWy * timeInterval;
        final double psdZ = varWz * timeInterval;

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

        final double normRootPsd = Math.sqrt(rootPsdX * rootPsdX
                + rootPsdY * rootPsdY + rootPsdZ * rootPsdZ);
        assertEquals(normRootPsd, estimator.getNoiseRootPsdNorm(), ABSOLUTE_ERROR);

        assertEquals(windowSize, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());

        assertEquals(mStart, 1);
        assertEquals(mTriadAdded, windowSize);
        assertEquals(mWindowFilled, 1);
        assertEquals(mReset, 0);

        // if we add more triads, window filled is not called again
        BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                errors, random, kinematics);
        kinematics.getAngularRateTriad(triad);

        triads.add(new AngularSpeedTriad(triad));

        assertTrue(estimator.addTriadAndProcess(
                triad.getValueX(), triad.getValueY(), triad.getValueZ()));

        assertEquals(windowSize + 1, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());

        assertEquals(triads.size(), windowSize + 1);
        assertEquals(triads.get(1), estimator.getFirstWindowedTriad());
        assertEquals(triads.get(windowSize), estimator.getLastWindowedTriad());

        assertEquals(mStart, 1);
        assertEquals(mTriadAdded, windowSize + 1);
        assertEquals(mWindowFilled, 1);
        assertEquals(mReset, 0);

        // reset
        assertTrue(estimator.reset());

        assertEquals(mReset, 1);

        assertNull(estimator.getFirstWindowedTriad());
        assertNull(estimator.getLastWindowedTriad());
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
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isWindowFilled());
    }

    @Test
    public void testAddTriadAndProcessAndThenReset3() throws WrongSizeException, LockedException {
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

        final WindowedAngularSpeedTriadNoiseEstimator estimator =
                new WindowedAngularSpeedTriadNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mTriadAdded, 0);
        assertEquals(mWindowFilled, 0);
        assertEquals(mReset, 0);
        assertFalse(estimator.isWindowFilled());
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getFirstWindowedTriad());
        assertNull(estimator.getLastWindowedTriad());
        assertFalse(estimator.isRunning());

        final BodyKinematics kinematics = new BodyKinematics();
        final BodyKinematics firstKinematics = new BodyKinematics();
        final int windowSize = estimator.getWindowSize();
        final double timeInterval = estimator.getTimeInterval();
        final AngularSpeedTriad firstTriad = new AngularSpeedTriad();
        final AngularSpeedTriad lastTriad = new AngularSpeedTriad();
        final AngularSpeedTriad triad = new AngularSpeedTriad();
        double valueX;
        double valueY;
        double valueZ;
        double avgWx = 0.0;
        double avgWy = 0.0;
        double avgWz = 0.0;
        final List<AngularSpeedTriad> triads = new ArrayList<>();
        for (int i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedTriad(firstTriad)) {
                assertEquals(estimator.getFirstWindowedTriad(), firstTriad);
                assertEquals(firstTriad, firstKinematics.getAngularRateTriad());
            }
            if (estimator.getLastWindowedTriad(lastTriad)) {
                assertEquals(estimator.getLastWindowedTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                    errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getAngularRateTriad(triad);
            triads.add(new AngularSpeedTriad(triad));
            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            assertEquals(estimator.addTriadAndProcess(
                    triad.getMeasurementX(), triad.getMeasurementY(), triad.getMeasurementZ()),
                    i != 0);

            assertTrue(estimator.getLastWindowedTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avgWx += valueX;
            avgWy += valueY;
            avgWz += valueZ;
        }

        avgWx /= windowSize;
        avgWy /= windowSize;
        avgWz /= windowSize;

        double varWx = 0.0;
        double varWy = 0.0;
        double varWz = 0.0;
        for (int i = 0; i < windowSize; i++) {
            triad.copyFrom(triads.get(i));

            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            final double diffX = valueX - avgWx;
            final double diffY = valueY - avgWy;
            final double diffZ = valueZ - avgWz;

            varWx += diffX * diffX;
            varWy += diffY * diffY;
            varWz += diffZ * diffZ;
        }

        varWx /= (windowSize - 1);
        varWy /= (windowSize - 1);
        varWz /= (windowSize - 1);

        final double stdWx = Math.sqrt(varWx);
        final double stdWy = Math.sqrt(varWy);
        final double stdWz = Math.sqrt(varWz);

        assertEquals(avgWx, estimator.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(avgWy, estimator.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(avgWz, estimator.getAvgZ(), ABSOLUTE_ERROR);

        AngularSpeed w1 = estimator.getAvgXAsMeasurement();
        assertEquals(avgWx, w1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        final AngularSpeed w2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgXAsMeasurement(w2);
        assertEquals(w1, w2);

        w1 = estimator.getAvgYAsMeasurement();
        assertEquals(avgWy, w1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        estimator.getAvgYAsMeasurement(w2);
        assertEquals(w1, w2);

        w1 = estimator.getAvgZAsMeasurement();
        assertEquals(avgWz, w1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, w1.getUnit());
        estimator.getAvgZAsMeasurement(w2);
        assertEquals(w1, w2);

        final AngularSpeedTriad triad1 = estimator.getAvgTriad();
        assertEquals(avgWx, triad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(avgWy, triad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(avgWz, triad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());
        final AngularSpeedTriad triad2 = new AngularSpeedTriad();
        estimator.getAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(triad1.getNorm(), estimator.getAvgNorm(), ABSOLUTE_ERROR);
        final AngularSpeed norm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(triad1.getNorm(), norm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, norm1.getUnit());
        final AngularSpeed norm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAvgNormAsMeasurement(norm2);
        assertEquals(norm1, norm2);

        assertEquals(varWx, estimator.getVarianceX(), ABSOLUTE_ERROR);
        assertEquals(varWy, estimator.getVarianceY(), ABSOLUTE_ERROR);
        assertEquals(varWz, estimator.getVarianceZ(), ABSOLUTE_ERROR);

        assertEquals(stdWx, estimator.getStandardDeviationX(), ABSOLUTE_ERROR);
        assertEquals(stdWy, estimator.getStandardDeviationY(), ABSOLUTE_ERROR);
        assertEquals(stdWz, estimator.getStandardDeviationZ(), ABSOLUTE_ERROR);

        final AngularSpeed stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(stdWx, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdX1.getUnit());
        final AngularSpeed stdX2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);

        final AngularSpeed stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(stdWy, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdY1.getUnit());
        final AngularSpeed stdY2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);

        final AngularSpeed stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(stdWz, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdZ1.getUnit());
        final AngularSpeed stdZ2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        final AngularSpeedTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(stdWx, stdTriad1.getValueX(), 0.0);
        assertEquals(stdWy, stdTriad1.getValueY(), 0.0);
        assertEquals(stdWz, stdTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdTriad1.getUnit());
        final AngularSpeedTriad stdTriad2 = new AngularSpeedTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);

        assertEquals(stdTriad1.getNorm(), estimator.getStandardDeviationNorm(), ABSOLUTE_ERROR);
        final AngularSpeed stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(stdTriad1.getNorm(), stdNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, stdNorm1.getUnit());
        final AngularSpeed stdNorm2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);

        final double avgStd = (stdWx + stdWy + stdWz) / 3.0;
        assertEquals(avgStd, estimator.getAverageStandardDeviation(), ABSOLUTE_ERROR);
        final AngularSpeed avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(avgStd, avgStd1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avgStd1.getUnit());
        final AngularSpeed avgStd2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);

        final double psdX = varWx * timeInterval;
        final double psdY = varWy * timeInterval;
        final double psdZ = varWz * timeInterval;

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

        final double normRootPsd = Math.sqrt(rootPsdX * rootPsdX
                + rootPsdY * rootPsdY + rootPsdZ * rootPsdZ);
        assertEquals(normRootPsd, estimator.getNoiseRootPsdNorm(), ABSOLUTE_ERROR);

        assertEquals(windowSize, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());

        assertEquals(mStart, 1);
        assertEquals(mTriadAdded, windowSize);
        assertEquals(mWindowFilled, 1);
        assertEquals(mReset, 0);

        // if we add more triads, window filled is not called again
        BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                errors, random, kinematics);
        kinematics.getAngularRateTriad(triad);

        triads.add(new AngularSpeedTriad(triad));

        assertTrue(estimator.addTriadAndProcess(
                triad.getMeasurementX(), triad.getMeasurementY(), triad.getMeasurementZ()));

        assertEquals(windowSize + 1, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());

        assertEquals(triads.size(), windowSize + 1);
        assertEquals(triads.get(1), estimator.getFirstWindowedTriad());
        assertEquals(triads.get(windowSize), estimator.getLastWindowedTriad());

        assertEquals(mStart, 1);
        assertEquals(mTriadAdded, windowSize + 1);
        assertEquals(mWindowFilled, 1);
        assertEquals(mReset, 0);

        // reset
        assertTrue(estimator.reset());

        assertEquals(mReset, 1);

        assertNull(estimator.getFirstWindowedTriad());
        assertNull(estimator.getLastWindowedTriad());
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
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isWindowFilled());
    }

    @Test
    public void testAddTriad1() throws WrongSizeException, LockedException {
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

        final WindowedAngularSpeedTriadNoiseEstimator estimator =
                new WindowedAngularSpeedTriadNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mTriadAdded, 0);
        assertEquals(mWindowFilled, 0);
        assertEquals(mReset, 0);
        assertFalse(estimator.isWindowFilled());
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getFirstWindowedTriad());
        assertNull(estimator.getLastWindowedTriad());
        assertFalse(estimator.isRunning());

        final BodyKinematics kinematics = new BodyKinematics();
        final BodyKinematics firstKinematics = new BodyKinematics();
        final int windowSize = estimator.getWindowSize();
        final double timeInterval = estimator.getTimeInterval();
        final AngularSpeedTriad firstTriad = new AngularSpeedTriad();
        final AngularSpeedTriad lastTriad = new AngularSpeedTriad();
        final AngularSpeedTriad triad = new AngularSpeedTriad();
        final List<AngularSpeedTriad> triads = new ArrayList<>();
        for (int i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedTriad(firstTriad)) {
                assertEquals(estimator.getFirstWindowedTriad(), firstTriad);
                assertEquals(firstTriad, firstKinematics.getAngularRateTriad());
            }
            if (estimator.getLastWindowedTriad(lastTriad)) {
                assertEquals(estimator.getLastWindowedTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                    errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getAngularRateTriad(triad);
            triads.add(new AngularSpeedTriad(triad));
            estimator.addTriad(triad);

            assertTrue(estimator.getLastWindowedTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(estimator.getNumberOfProcessedSamples(), 0);
            assertFalse(estimator.isRunning());
        }

        assertEquals(mStart, 1);
        assertEquals(mTriadAdded, windowSize);
        assertEquals(mWindowFilled, 1);
        assertEquals(mReset, 0);

        assertEquals(triads.size(), windowSize);
        assertEquals(triads.get(0), estimator.getFirstWindowedTriad());
        assertEquals(triads.get(windowSize - 1), estimator.getLastWindowedTriad());
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
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());
    }

    @Test
    public void testAddTriad2() throws WrongSizeException, LockedException {
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

        final WindowedAngularSpeedTriadNoiseEstimator estimator =
                new WindowedAngularSpeedTriadNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mTriadAdded, 0);
        assertEquals(mWindowFilled, 0);
        assertEquals(mReset, 0);
        assertFalse(estimator.isWindowFilled());
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getFirstWindowedTriad());
        assertNull(estimator.getLastWindowedTriad());
        assertFalse(estimator.isRunning());

        final BodyKinematics kinematics = new BodyKinematics();
        final BodyKinematics firstKinematics = new BodyKinematics();
        final int windowSize = estimator.getWindowSize();
        final double timeInterval = estimator.getTimeInterval();
        final AngularSpeedTriad firstTriad = new AngularSpeedTriad();
        final AngularSpeedTriad lastTriad = new AngularSpeedTriad();
        final AngularSpeedTriad triad = new AngularSpeedTriad();
        final List<AngularSpeedTriad> triads = new ArrayList<>();
        for (int i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedTriad(firstTriad)) {
                assertEquals(estimator.getFirstWindowedTriad(), firstTriad);
                assertEquals(firstTriad, firstKinematics.getAngularRateTriad());
            }
            if (estimator.getLastWindowedTriad(lastTriad)) {
                assertEquals(estimator.getLastWindowedTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                    errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getAngularRateTriad(triad);
            triads.add(new AngularSpeedTriad(triad));
            estimator.addTriad(triad.getValueX(), triad.getValueY(), triad.getValueZ());

            assertTrue(estimator.getLastWindowedTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(estimator.getNumberOfProcessedSamples(), 0);
            assertFalse(estimator.isRunning());
        }

        assertEquals(mStart, 1);
        assertEquals(mTriadAdded, windowSize);
        assertEquals(mWindowFilled, 1);
        assertEquals(mReset, 0);

        assertEquals(triads.size(), windowSize);
        assertEquals(triads.get(0), estimator.getFirstWindowedTriad());
        assertEquals(triads.get(windowSize - 1), estimator.getLastWindowedTriad());
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
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());
    }

    @Test
    public void testAddTriad3() throws WrongSizeException, LockedException {
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

        final WindowedAngularSpeedTriadNoiseEstimator estimator =
                new WindowedAngularSpeedTriadNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mTriadAdded, 0);
        assertEquals(mWindowFilled, 0);
        assertEquals(mReset, 0);
        assertFalse(estimator.isWindowFilled());
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getFirstWindowedTriad());
        assertNull(estimator.getLastWindowedTriad());
        assertFalse(estimator.isRunning());

        final BodyKinematics kinematics = new BodyKinematics();
        final BodyKinematics firstKinematics = new BodyKinematics();
        final int windowSize = estimator.getWindowSize();
        final double timeInterval = estimator.getTimeInterval();
        final AngularSpeedTriad firstTriad = new AngularSpeedTriad();
        final AngularSpeedTriad lastTriad = new AngularSpeedTriad();
        final AngularSpeedTriad triad = new AngularSpeedTriad();
        final List<AngularSpeedTriad> triads = new ArrayList<>();
        for (int i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedTriad(firstTriad)) {
                assertEquals(estimator.getFirstWindowedTriad(), firstTriad);
                assertEquals(firstTriad, firstKinematics.getAngularRateTriad());
            }
            if (estimator.getLastWindowedTriad(lastTriad)) {
                assertEquals(estimator.getLastWindowedTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                    errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getAngularRateTriad(triad);
            triads.add(new AngularSpeedTriad(triad));
            estimator.addTriad(
                    triad.getMeasurementX(), triad.getMeasurementY(), triad.getMeasurementZ());

            assertTrue(estimator.getLastWindowedTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(estimator.getNumberOfProcessedSamples(), 0);
            assertFalse(estimator.isRunning());
        }

        assertEquals(mStart, 1);
        assertEquals(mTriadAdded, windowSize);
        assertEquals(mWindowFilled, 1);
        assertEquals(mReset, 0);

        assertEquals(triads.size(), windowSize);
        assertEquals(triads.get(0), estimator.getFirstWindowedTriad());
        assertEquals(triads.get(windowSize - 1), estimator.getLastWindowedTriad());
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
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());
    }

    @Override
    public void onStart(final WindowedAngularSpeedTriadNoiseEstimator estimator) {
        checkLocked(estimator);
        mStart++;
    }

    @Override
    public void onTriadAdded(final WindowedAngularSpeedTriadNoiseEstimator estimator) {
        mTriadAdded++;
    }

    @Override
    public void onWindowFilled(final WindowedAngularSpeedTriadNoiseEstimator estimator) {
        mWindowFilled++;
    }

    @Override
    public void onReset(final WindowedAngularSpeedTriadNoiseEstimator estimator) {
        mReset++;
    }

    private void reset() {
        mStart = 0;
        mTriadAdded = 0;
        mWindowFilled = 0;
        mReset = 0;
    }

    private void checkLocked(final WindowedAngularSpeedTriadNoiseEstimator estimator) {
        assertTrue(estimator.isRunning());
        try {
            estimator.setWindowSize(3);
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
            estimator.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.addTriadAndProcess(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.addTriadAndProcess(0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        final AngularSpeed w = new AngularSpeed(0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        try {
            estimator.addTriadAndProcess(w, w, w);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.addTriad(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.addTriad(0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
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
