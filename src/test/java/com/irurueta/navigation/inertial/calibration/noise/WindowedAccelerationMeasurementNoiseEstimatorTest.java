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
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class WindowedAccelerationMeasurementNoiseEstimatorTest implements
        WindowedAccelerationMeasurementNoiseEstimatorListener{

    private static final double MIN_ACCELEROMETER_VALUE = -2.0 * 9.81;
    private static final double MAX_ACCELEROMETER_VALUE = 2.0 * 9.81;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private int mStart;
    private int mMeasurementAdded;
    private int mWindowFilled;
    private int mReset;

    @Test
    public void testConstructor1() {
        final WindowedAccelerationMeasurementNoiseEstimator estimator =
                new WindowedAccelerationMeasurementNoiseEstimator();

        // check default values
        assertEquals(WindowedAccelerationMeasurementNoiseEstimator.DEFAULT_WINDOW_SIZE,
                estimator.getWindowSize());
        assertEquals(WindowedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertNull(estimator.getListener());
        assertNull(estimator.getFirstWindowedMeasurementValue());
        assertNull(estimator.getLastWindowedMeasurementValue());
        assertNull(estimator.getFirstWindowedMeasurement());
        assertFalse(estimator.getFirstWindowedMeasurement(null));
        assertNull(estimator.getLastWindowedMeasurement());
        assertFalse(estimator.getLastWindowedMeasurement(null));
        assertEquals(0.0, estimator.getAvg(), 0.0);
        final Acceleration avg1 = estimator.getAvgAsMeasurement();
        assertEquals(0.0, avg1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avg1.getUnit());
        final Acceleration avg2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        final Acceleration std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(0.0, std1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, std1.getUnit());
        final Acceleration std2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfAddedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isWindowFilled());
        assertFalse(estimator.isRunning());
    }

    @Test
    public void testConstructor2() {
        final WindowedAccelerationMeasurementNoiseEstimator estimator =
                new WindowedAccelerationMeasurementNoiseEstimator(this);

        // check default values
        assertEquals(WindowedAccelerationMeasurementNoiseEstimator.DEFAULT_WINDOW_SIZE,
                estimator.getWindowSize());
        assertEquals(WindowedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getFirstWindowedMeasurementValue());
        assertNull(estimator.getLastWindowedMeasurementValue());
        assertNull(estimator.getFirstWindowedMeasurement());
        assertFalse(estimator.getFirstWindowedMeasurement(null));
        assertNull(estimator.getLastWindowedMeasurement());
        assertFalse(estimator.getLastWindowedMeasurement(null));
        assertEquals(0.0, estimator.getAvg(), 0.0);
        final Acceleration avg1 = estimator.getAvgAsMeasurement();
        assertEquals(0.0, avg1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avg1.getUnit());
        final Acceleration avg2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        final Acceleration std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(0.0, std1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, std1.getUnit());
        final Acceleration std2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfAddedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isWindowFilled());
        assertFalse(estimator.isRunning());
    }

    @Test
    public void testGetSetWindowSize() throws LockedException {
        final WindowedAccelerationMeasurementNoiseEstimator estimator =
                new WindowedAccelerationMeasurementNoiseEstimator();

        // check default value
        assertEquals(WindowedAccelerationMeasurementNoiseEstimator.DEFAULT_WINDOW_SIZE,
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
        try {
            estimator.setWindowSize(2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetTimeInterval() throws LockedException {
        final WindowedAccelerationMeasurementNoiseEstimator estimator =
                new WindowedAccelerationMeasurementNoiseEstimator();

        // check default value
        assertEquals(WindowedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final WindowedAccelerationMeasurementNoiseEstimator estimator =
                new WindowedAccelerationMeasurementNoiseEstimator();

        // check default value
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final WindowedAccelerationMeasurementNoiseEstimator estimator =
                new WindowedAccelerationMeasurementNoiseEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testAddMeasurementAndProcessAndThenReset1()
            throws WrongSizeException, LockedException {
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
        final double fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double omegaX = 0.0;
        final double omegaY = 0.0;
        final double omegaZ = 0.0;

        final BodyKinematics trueKinematics = new BodyKinematics(fx, fy, fz,
                omegaX, omegaY, omegaZ);

        final WindowedAccelerationMeasurementNoiseEstimator estimator =
                new WindowedAccelerationMeasurementNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mMeasurementAdded, 0);
        assertEquals(mWindowFilled, 0);
        assertEquals(mReset, 0);
        assertFalse(estimator.isWindowFilled());
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertEquals(estimator.getNumberOfAddedSamples(), 0);
        assertEquals(estimator.getNumberOfSamplesInWindow(), 0);
        assertNull(estimator.getFirstWindowedMeasurementValue());
        assertNull(estimator.getFirstWindowedMeasurement());
        assertFalse(estimator.getFirstWindowedMeasurement(null));
        assertNull(estimator.getLastWindowedMeasurementValue());
        assertNull(estimator.getLastWindowedMeasurement());
        assertFalse(estimator.getLastWindowedMeasurement(null));
        assertFalse(estimator.isRunning());

        final BodyKinematics kinematics = new BodyKinematics();
        final BodyKinematics firstKinematics = new BodyKinematics();
        final int windowSize = estimator.getWindowSize();
        final double timeInterval = estimator.getTimeInterval();
        final Acceleration firstMeasurement = new Acceleration(
                0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration lastMeasurement = new Acceleration(
                0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration measurement = new Acceleration(
                0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        double value;
        double avg = 0.0;
        final List<Acceleration> measurements = new ArrayList<>();
        for (int i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedMeasurement(firstMeasurement)) {
                assertEquals(estimator.getFirstWindowedMeasurement(), firstMeasurement);
                assertEquals(estimator.getFirstWindowedMeasurementValue(),
                        firstMeasurement.getValue().doubleValue(), 0.0);
                assertEquals(firstMeasurement, firstKinematics.getSpecificForceNormAsAcceleration());
            }
            if (estimator.getLastWindowedMeasurement(lastMeasurement)) {
                assertEquals(estimator.getLastWindowedMeasurement(), lastMeasurement);
                assertEquals(estimator.getLastWindowedMeasurementValue(),
                        lastMeasurement.getValue().doubleValue(), 0.0);
                assertEquals(lastMeasurement, measurement);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                    errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getSpecificForceNormAsAcceleration(measurement);
            measurements.add(new Acceleration(measurement.getValue(), measurement.getUnit()));
            value = measurement.getValue().doubleValue();

            estimator.addMeasurementAndProcess(value);

            assertTrue(estimator.getLastWindowedMeasurement(lastMeasurement));
            assertEquals(lastMeasurement, measurement);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertEquals(estimator.getNumberOfAddedSamples(), i + 1);
            assertEquals(estimator.getNumberOfSamplesInWindow(), i + 1);
            assertFalse(estimator.isRunning());

            avg += value;
        }

        avg /= windowSize;

        double var = 0.0;
        for (int i = 0; i < windowSize; i++) {
            measurement.setValue(measurements.get(i).getValue());
            measurement.setUnit(measurements.get(i).getUnit());

            value = measurement.getValue().doubleValue();

            final double diff = value - avg;

            var += diff * diff;
        }

        var /= (windowSize - 1);

        final double std = Math.sqrt(var);

        assertEquals(avg, estimator.getAvg(), ABSOLUTE_ERROR);

        final Acceleration a1 = estimator.getAvgAsMeasurement();
        assertEquals(avg, a1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final Acceleration a2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgAsMeasurement(a2);
        assertEquals(a1, a2);

        assertEquals(var, estimator.getVariance(), ABSOLUTE_ERROR);
        assertEquals(std, estimator.getStandardDeviation(), ABSOLUTE_ERROR);

        final Acceleration std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(std, std1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, std1.getUnit());
        final Acceleration std2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);

        final double psd = var * timeInterval;
        assertEquals(psd, estimator.getPsd(), ABSOLUTE_ERROR);

        final double rootPsd = Math.sqrt(psd);
        assertEquals(rootPsd, estimator.getRootPsd(), ABSOLUTE_ERROR);
        assertEquals(estimator.getRootPsd(),
                estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);

        assertEquals(windowSize, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize, estimator.getNumberOfAddedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());

        assertEquals(mStart, 1);
        assertEquals(mMeasurementAdded, windowSize);
        assertEquals(mWindowFilled, 1);
        assertEquals(mReset, 0);

        // if we add more measurements, window filled is not called again
        BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                errors, random, kinematics);
        kinematics.getSpecificForceNormAsAcceleration(measurement);

        measurements.add(new Acceleration(measurement.getValue(), measurement.getUnit()));

        value = measurement.getValue().doubleValue();
        estimator.addMeasurementAndProcess(value);

        assertEquals(windowSize + 1, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize + 1, estimator.getNumberOfAddedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertTrue(estimator.isWindowFilled());

        assertEquals(measurements.size(), windowSize + 1);
        assertEquals(measurements.get(1), estimator.getFirstWindowedMeasurement());
        assertEquals(measurements.get(windowSize), estimator.getLastWindowedMeasurement());

        assertEquals(mStart, 1);
        assertEquals(mMeasurementAdded, windowSize + 1);
        assertEquals(mWindowFilled, 1);
        assertEquals(mReset, 0);

        // reset
        assertTrue(estimator.reset());

        assertEquals(mReset, 1);

        assertNull(estimator.getFirstWindowedMeasurementValue());
        assertNull(estimator.getFirstWindowedMeasurement());
        assertFalse(estimator.getFirstWindowedMeasurement(null));
        assertNull(estimator.getLastWindowedMeasurementValue());
        assertNull(estimator.getLastWindowedMeasurement());
        assertFalse(estimator.getLastWindowedMeasurement(null));
        assertEquals(0.0, estimator.getAvg(), 0.0);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfAddedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isWindowFilled());
    }

    @Test
    public void testAddMeasurementAndProcessAndThenReset2()
            throws WrongSizeException, LockedException {
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
        final double fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double omegaX = 0.0;
        final double omegaY = 0.0;
        final double omegaZ = 0.0;

        final BodyKinematics trueKinematics = new BodyKinematics(fx, fy, fz,
                omegaX, omegaY, omegaZ);

        final WindowedAccelerationMeasurementNoiseEstimator estimator =
                new WindowedAccelerationMeasurementNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mMeasurementAdded, 0);
        assertEquals(mWindowFilled, 0);
        assertEquals(mReset, 0);
        assertFalse(estimator.isWindowFilled());
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertEquals(estimator.getNumberOfAddedSamples(), 0);
        assertEquals(estimator.getNumberOfSamplesInWindow(), 0);
        assertNull(estimator.getFirstWindowedMeasurementValue());
        assertNull(estimator.getFirstWindowedMeasurement());
        assertFalse(estimator.getFirstWindowedMeasurement(null));
        assertNull(estimator.getLastWindowedMeasurementValue());
        assertNull(estimator.getLastWindowedMeasurement());
        assertFalse(estimator.getLastWindowedMeasurement(null));
        assertFalse(estimator.isRunning());

        final BodyKinematics kinematics = new BodyKinematics();
        final BodyKinematics firstKinematics = new BodyKinematics();
        final int windowSize = estimator.getWindowSize();
        final double timeInterval = estimator.getTimeInterval();
        final Acceleration firstMeasurement = new Acceleration(
                0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration lastMeasurement = new Acceleration(
                0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration measurement = new Acceleration(
                0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        double value;
        double avg = 0.0;
        final List<Acceleration> measurements = new ArrayList<>();
        for (int i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedMeasurement(firstMeasurement)) {
                assertEquals(estimator.getFirstWindowedMeasurement(), firstMeasurement);
                assertEquals(estimator.getFirstWindowedMeasurementValue(),
                        firstMeasurement.getValue().doubleValue(), 0.0);
                assertEquals(firstMeasurement, firstKinematics.getSpecificForceNormAsAcceleration());
            }
            if (estimator.getLastWindowedMeasurement(lastMeasurement)) {
                assertEquals(estimator.getLastWindowedMeasurement(), lastMeasurement);
                assertEquals(estimator.getLastWindowedMeasurementValue(),
                        lastMeasurement.getValue().doubleValue(), 0.0);
                assertEquals(lastMeasurement, measurement);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                    errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getSpecificForceNormAsAcceleration(measurement);
            measurements.add(new Acceleration(measurement.getValue(), measurement.getUnit()));
            value = measurement.getValue().doubleValue();

            estimator.addMeasurementAndProcess(measurement);

            assertTrue(estimator.getLastWindowedMeasurement(lastMeasurement));
            assertEquals(lastMeasurement, measurement);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertEquals(estimator.getNumberOfAddedSamples(), i + 1);
            assertEquals(estimator.getNumberOfSamplesInWindow(), i + 1);
            assertFalse(estimator.isRunning());

            avg += value;
        }

        avg /= windowSize;

        double var = 0.0;
        for (int i = 0; i < windowSize; i++) {
            measurement.setValue(measurements.get(i).getValue());
            measurement.setUnit(measurements.get(i).getUnit());

            value = measurement.getValue().doubleValue();

            final double diff = value - avg;

            var += diff * diff;
        }

        var /= (windowSize - 1);

        final double std = Math.sqrt(var);

        assertEquals(avg, estimator.getAvg(), ABSOLUTE_ERROR);

        final Acceleration a1 = estimator.getAvgAsMeasurement();
        assertEquals(avg, a1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, a1.getUnit());
        final Acceleration a2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgAsMeasurement(a2);
        assertEquals(a1, a2);

        assertEquals(var, estimator.getVariance(), ABSOLUTE_ERROR);
        assertEquals(std, estimator.getStandardDeviation(), ABSOLUTE_ERROR);

        final Acceleration std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(std, std1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, std1.getUnit());
        final Acceleration std2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);

        final double psd = var * timeInterval;
        assertEquals(psd, estimator.getPsd(), ABSOLUTE_ERROR);

        final double rootPsd = Math.sqrt(psd);
        assertEquals(rootPsd, estimator.getRootPsd(), ABSOLUTE_ERROR);
        assertEquals(estimator.getRootPsd(),
                estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);

        assertEquals(windowSize, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize, estimator.getNumberOfAddedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());

        assertEquals(mStart, 1);
        assertEquals(mMeasurementAdded, windowSize);
        assertEquals(mWindowFilled, 1);
        assertEquals(mReset, 0);

        // if we add more measurements, window filled is not called again
        BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                errors, random, kinematics);
        kinematics.getSpecificForceNormAsAcceleration(measurement);

        measurements.add(new Acceleration(measurement.getValue(), measurement.getUnit()));

        estimator.addMeasurementAndProcess(measurement);

        assertEquals(windowSize + 1, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize + 1, estimator.getNumberOfAddedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertTrue(estimator.isWindowFilled());

        assertEquals(measurements.size(), windowSize + 1);
        assertEquals(measurements.get(1), estimator.getFirstWindowedMeasurement());
        assertEquals(measurements.get(windowSize), estimator.getLastWindowedMeasurement());

        assertEquals(mStart, 1);
        assertEquals(mMeasurementAdded, windowSize + 1);
        assertEquals(mWindowFilled, 1);
        assertEquals(mReset, 0);

        // reset
        assertTrue(estimator.reset());

        assertEquals(mReset, 1);

        assertNull(estimator.getFirstWindowedMeasurementValue());
        assertNull(estimator.getFirstWindowedMeasurement());
        assertFalse(estimator.getFirstWindowedMeasurement(null));
        assertNull(estimator.getLastWindowedMeasurementValue());
        assertNull(estimator.getLastWindowedMeasurement());
        assertFalse(estimator.getLastWindowedMeasurement(null));
        assertEquals(0.0, estimator.getAvg(), 0.0);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(estimator.getRootPsd(), estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfAddedSamples());
        assertEquals(0, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isWindowFilled());
    }

    @Test
    public void testAddMeasurement1() throws WrongSizeException, LockedException {
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
        final double fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double omegaX = 0.0;
        final double omegaY = 0.0;
        final double omegaZ = 0.0;

        final BodyKinematics trueKinematics = new BodyKinematics(fx, fy, fz,
                omegaX, omegaY, omegaZ);

        final WindowedAccelerationMeasurementNoiseEstimator estimator =
                new WindowedAccelerationMeasurementNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mMeasurementAdded, 0);
        assertEquals(mWindowFilled, 0);
        assertEquals(mReset, 0);
        assertFalse(estimator.isWindowFilled());
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertEquals(estimator.getNumberOfAddedSamples(), 0);
        assertEquals(estimator.getNumberOfSamplesInWindow(), 0);
        assertNull(estimator.getFirstWindowedMeasurementValue());
        assertNull(estimator.getFirstWindowedMeasurement());
        assertFalse(estimator.getFirstWindowedMeasurement(null));
        assertNull(estimator.getLastWindowedMeasurementValue());
        assertNull(estimator.getLastWindowedMeasurement());
        assertFalse(estimator.getLastWindowedMeasurement(null));
        assertFalse(estimator.isRunning());

        final BodyKinematics kinematics = new BodyKinematics();
        final BodyKinematics firstKinematics = new BodyKinematics();
        final int windowSize = estimator.getWindowSize();
        final double timeInterval = estimator.getTimeInterval();
        final Acceleration firstMeasurement = new Acceleration(
                0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration lastMeasurement = new Acceleration(
                0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration measurement = new Acceleration(
                0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        double value;
        final List<Acceleration> measurements = new ArrayList<>();
        for (int i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedMeasurement(firstMeasurement)) {
                assertEquals(estimator.getFirstWindowedMeasurement(), firstMeasurement);
                assertEquals(estimator.getFirstWindowedMeasurementValue(),
                        firstMeasurement.getValue().doubleValue(), 0.0);
                assertEquals(firstMeasurement, firstKinematics.getSpecificForceNormAsAcceleration());
            }
            if (estimator.getLastWindowedMeasurement(lastMeasurement)) {
                assertEquals(estimator.getLastWindowedMeasurement(), lastMeasurement);
                assertEquals(estimator.getLastWindowedMeasurementValue(),
                        lastMeasurement.getValue().doubleValue(), 0.0);
                assertEquals(lastMeasurement, measurement);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                    errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getSpecificForceNormAsAcceleration(measurement);
            measurements.add(new Acceleration(measurement.getValue(), measurement.getUnit()));
            value = measurement.getValue().doubleValue();

            estimator.addMeasurement(value);

            assertTrue(estimator.getLastWindowedMeasurement(lastMeasurement));
            assertEquals(lastMeasurement, measurement);
            assertEquals(estimator.getNumberOfProcessedSamples(), 0);
            assertEquals(estimator.getNumberOfAddedSamples(), i + 1);
            assertEquals(estimator.getNumberOfSamplesInWindow(), i + 1);
            assertFalse(estimator.isRunning());
        }

        assertEquals(mStart, 1);
        assertEquals(mMeasurementAdded, windowSize);
        assertEquals(mWindowFilled, 1);
        assertEquals(mReset, 0);

        assertEquals(measurements.size(), windowSize);
        assertEquals(measurements.get(0), estimator.getFirstWindowedMeasurement());
        assertTrue(estimator.getFirstWindowedMeasurement(firstMeasurement));
        assertEquals(measurements.get(0), firstMeasurement);
        assertEquals(firstMeasurement.getValue().doubleValue(),
                estimator.getFirstWindowedMeasurementValue(), 0.0);
        assertEquals(measurements.get(windowSize - 1), estimator.getLastWindowedMeasurement());
        assertTrue(estimator.getLastWindowedMeasurement(lastMeasurement));
        assertEquals(measurements.get(windowSize - 1), lastMeasurement);
        assertEquals(lastMeasurement.getValue().doubleValue(),
                estimator.getLastWindowedMeasurementValue(), 0.0);
        assertEquals(0.0, estimator.getAvg(), 0.0);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize, estimator.getNumberOfAddedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());
    }

    @Test
    public void testAddMeasurement2() throws WrongSizeException, LockedException {
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
        final double fx = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fy = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double fz = randomizer.nextDouble(MIN_ACCELEROMETER_VALUE,
                MAX_ACCELEROMETER_VALUE);
        final double omegaX = 0.0;
        final double omegaY = 0.0;
        final double omegaZ = 0.0;

        final BodyKinematics trueKinematics = new BodyKinematics(fx, fy, fz,
                omegaX, omegaY, omegaZ);

        final WindowedAccelerationMeasurementNoiseEstimator estimator =
                new WindowedAccelerationMeasurementNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mMeasurementAdded, 0);
        assertEquals(mWindowFilled, 0);
        assertEquals(mReset, 0);
        assertFalse(estimator.isWindowFilled());
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertEquals(estimator.getNumberOfAddedSamples(), 0);
        assertEquals(estimator.getNumberOfSamplesInWindow(), 0);
        assertNull(estimator.getFirstWindowedMeasurementValue());
        assertNull(estimator.getFirstWindowedMeasurement());
        assertFalse(estimator.getFirstWindowedMeasurement(null));
        assertNull(estimator.getLastWindowedMeasurementValue());
        assertNull(estimator.getLastWindowedMeasurement());
        assertFalse(estimator.getLastWindowedMeasurement(null));
        assertFalse(estimator.isRunning());

        final BodyKinematics kinematics = new BodyKinematics();
        final BodyKinematics firstKinematics = new BodyKinematics();
        final int windowSize = estimator.getWindowSize();
        final double timeInterval = estimator.getTimeInterval();
        final Acceleration firstMeasurement = new Acceleration(
                0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration lastMeasurement = new Acceleration(
                0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration measurement = new Acceleration(
                0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final List<Acceleration> measurements = new ArrayList<>();
        for (int i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedMeasurement(firstMeasurement)) {
                assertEquals(estimator.getFirstWindowedMeasurement(), firstMeasurement);
                assertEquals(estimator.getFirstWindowedMeasurementValue(),
                        firstMeasurement.getValue().doubleValue(), 0.0);
                assertEquals(firstMeasurement, firstKinematics.getSpecificForceNormAsAcceleration());
            }
            if (estimator.getLastWindowedMeasurement(lastMeasurement)) {
                assertEquals(estimator.getLastWindowedMeasurement(), lastMeasurement);
                assertEquals(estimator.getLastWindowedMeasurementValue(),
                        lastMeasurement.getValue().doubleValue(), 0.0);
                assertEquals(lastMeasurement, measurement);
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                    errors, random, kinematics);

            if (i == 0) {
                firstKinematics.copyFrom(kinematics);
            }

            kinematics.getSpecificForceNormAsAcceleration(measurement);
            measurements.add(new Acceleration(measurement.getValue(), measurement.getUnit()));

            estimator.addMeasurement(measurement);

            assertTrue(estimator.getLastWindowedMeasurement(lastMeasurement));
            assertEquals(lastMeasurement, measurement);
            assertEquals(estimator.getNumberOfProcessedSamples(), 0);
            assertEquals(estimator.getNumberOfAddedSamples(), i + 1);
            assertEquals(estimator.getNumberOfSamplesInWindow(), i + 1);
            assertFalse(estimator.isRunning());
        }

        assertEquals(mStart, 1);
        assertEquals(mMeasurementAdded, windowSize);
        assertEquals(mWindowFilled, 1);
        assertEquals(mReset, 0);

        assertEquals(measurements.size(), windowSize);
        assertEquals(measurements.get(0), estimator.getFirstWindowedMeasurement());
        assertTrue(estimator.getFirstWindowedMeasurement(firstMeasurement));
        assertEquals(measurements.get(0), firstMeasurement);
        assertEquals(firstMeasurement.getValue().doubleValue(),
                estimator.getFirstWindowedMeasurementValue(), 0.0);
        assertEquals(measurements.get(windowSize - 1), estimator.getLastWindowedMeasurement());
        assertTrue(estimator.getLastWindowedMeasurement(lastMeasurement));
        assertEquals(measurements.get(windowSize - 1), lastMeasurement);
        assertEquals(lastMeasurement.getValue().doubleValue(),
                estimator.getLastWindowedMeasurementValue(), 0.0);
        assertEquals(0.0, estimator.getAvg(), 0.0);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0.0, estimator.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(windowSize, estimator.getNumberOfAddedSamples());
        assertEquals(windowSize, estimator.getNumberOfSamplesInWindow());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isWindowFilled());
    }

    @Override
    public void onStart(final WindowedAccelerationMeasurementNoiseEstimator estimator) {
        checkLocked(estimator);
        mStart++;
    }

    @Override
    public void onMeasurementAdded(
            final WindowedAccelerationMeasurementNoiseEstimator estimator) {
        mMeasurementAdded++;
    }

    @Override
    public void onWindowFilled(final WindowedAccelerationMeasurementNoiseEstimator estimator) {
        mWindowFilled++;
    }

    @Override
    public void onReset(final WindowedAccelerationMeasurementNoiseEstimator estimator) {
        mReset++;
    }

    private void reset() {
        mStart = 0;
        mMeasurementAdded = 0;
        mWindowFilled = 0;
        mReset = 0;
    }

    private void checkLocked(final WindowedAccelerationMeasurementNoiseEstimator estimator) {
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
            estimator.addMeasurementAndProcess(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        final Acceleration a = new Acceleration(
                0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        try {
            estimator.addMeasurementAndProcess(a);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.addMeasurement(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.addMeasurement(a);
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
