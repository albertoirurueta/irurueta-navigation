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
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class AccumulatedAccelerationMeasurementNoiseEstimatorTest implements
        AccumulatedAccelerationMeasurementNoiseEstimatorListener {

    private static final double MIN_ACCELEROMETER_VALUE = -2.0 * 9.81;
    private static final double MAX_ACCELEROMETER_VALUE = 2.0 * 9.81;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double ABSOLUTE_ERROR = 1e-8;

    private static final int N_SAMPLES = 1000;

    private int mStart;
    private int mMeasurementAdded;
    private int mReset;

    @Test
    public void testConstructor1() {
        final AccumulatedAccelerationMeasurementNoiseEstimator estimator =
                new AccumulatedAccelerationMeasurementNoiseEstimator();

        // check default values
        assertEquals(AccumulatedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
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
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Test
    public void testConstructor2() {
        final AccumulatedAccelerationMeasurementNoiseEstimator estimator =
                new AccumulatedAccelerationMeasurementNoiseEstimator(this);

        // check default values
        assertEquals(AccumulatedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
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
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Test
    public void testGetSetTimeInterval() throws LockedException {
        final AccumulatedAccelerationMeasurementNoiseEstimator estimator =
                new AccumulatedAccelerationMeasurementNoiseEstimator();

        // check default value
        assertEquals(
                AccumulatedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final AccumulatedAccelerationMeasurementNoiseEstimator estimator =
                new AccumulatedAccelerationMeasurementNoiseEstimator();

        // check default value
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(
                AccumulatedAccelerationMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final AccumulatedAccelerationMeasurementNoiseEstimator estimator =
                new AccumulatedAccelerationMeasurementNoiseEstimator();

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

        final AccumulatedAccelerationMeasurementNoiseEstimator estimator =
                new AccumulatedAccelerationMeasurementNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mMeasurementAdded, 0);
        assertEquals(mReset, 0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertFalse(estimator.isRunning());

        final AccelerationTriad triad = new AccelerationTriad();
        final BodyKinematics kinematics = new BodyKinematics();
        final double timeInterval = estimator.getTimeInterval();
        final Acceleration lastMeasurement = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        double value;
        double avg = 0.0;
        double var = 0.0;
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastMeasurement(lastMeasurement)) {
                assertEquals(estimator.getLastMeasurement(), lastMeasurement);
                assertEquals(lastMeasurement, triad.getMeasurementNorm());
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                    errors, random, kinematics);
            kinematics.getSpecificForceTriad(triad);

            value = triad.getNorm();

            assertTrue(estimator.addMeasurement(value));

            assertTrue(estimator.getLastMeasurement(lastMeasurement));
            assertEquals(lastMeasurement, triad.getMeasurementNorm());
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avg = avg * (double) i / (double) j + value / j;

            final double diff = value - avg;

            final double diff2 = diff * diff;

            var = var * (double) i / (double) j + diff2 / j;
        }

        assertEquals(estimator.getNumberOfProcessedSamples(), N_SAMPLES);
        assertFalse(estimator.isRunning());
        assertEquals(mStart, 1);
        assertEquals(mMeasurementAdded, N_SAMPLES);
        assertEquals(mReset, 0);

        assertEquals(avg, estimator.getAvg(), ABSOLUTE_ERROR);

        final Acceleration avg1 = estimator.getAvgAsMeasurement();
        assertEquals(avg, avg1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avg1.getUnit());
        final Acceleration avg2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);

        assertEquals(var, estimator.getVariance(), ABSOLUTE_ERROR);

        final double std = Math.sqrt(var);

        assertEquals(std, estimator.getStandardDeviation(), ABSOLUTE_ERROR);

        final Acceleration std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(std, std1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, std1.getUnit());
        final Acceleration std2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);

        final double psd = timeInterval * var;

        assertEquals(psd, estimator.getPsd(), ABSOLUTE_ERROR);

        final double rootPsd = Math.sqrt(psd);

        assertEquals(rootPsd, estimator.getRootPsd(), ABSOLUTE_ERROR);

        // reset
        assertTrue(estimator.reset());

        assertEquals(mReset, 1);

        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertEquals(0.0, estimator.getAvg(), 0.0);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
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

        final AccumulatedAccelerationMeasurementNoiseEstimator estimator =
                new AccumulatedAccelerationMeasurementNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mMeasurementAdded, 0);
        assertEquals(mReset, 0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertFalse(estimator.isRunning());

        final AccelerationTriad triad = new AccelerationTriad();
        final BodyKinematics kinematics = new BodyKinematics();
        final double timeInterval = estimator.getTimeInterval();
        final Acceleration lastMeasurement = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        double value;
        double avg = 0.0;
        double var = 0.0;
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastMeasurement(lastMeasurement)) {
                assertEquals(estimator.getLastMeasurement(), lastMeasurement);
                assertEquals(lastMeasurement, triad.getMeasurementNorm());
            }

            BodyKinematicsGenerator.generate(timeInterval, trueKinematics,
                    errors, random, kinematics);
            kinematics.getSpecificForceTriad(triad);

            value = triad.getNorm();

            assertTrue(estimator.addMeasurement(triad.getMeasurementNorm()));

            assertTrue(estimator.getLastMeasurement(lastMeasurement));
            assertEquals(lastMeasurement, triad.getMeasurementNorm());
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avg = avg * (double) i / (double) j + value / j;

            final double diff = value - avg;

            final double diff2 = diff * diff;

            var = var * (double) i / (double) j + diff2 / j;
        }

        assertEquals(estimator.getNumberOfProcessedSamples(), N_SAMPLES);
        assertFalse(estimator.isRunning());
        assertEquals(mStart, 1);
        assertEquals(mMeasurementAdded, N_SAMPLES);
        assertEquals(mReset, 0);

        assertEquals(avg, estimator.getAvg(), ABSOLUTE_ERROR);

        final Acceleration avg1 = estimator.getAvgAsMeasurement();
        assertEquals(avg, avg1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avg1.getUnit());
        final Acceleration avg2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);

        assertEquals(var, estimator.getVariance(), ABSOLUTE_ERROR);

        final double std = Math.sqrt(var);

        assertEquals(std, estimator.getStandardDeviation(), ABSOLUTE_ERROR);

        final Acceleration std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(std, std1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, std1.getUnit());
        final Acceleration std2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);

        final double psd = timeInterval * var;

        assertEquals(psd, estimator.getPsd(), ABSOLUTE_ERROR);

        final double rootPsd = Math.sqrt(psd);

        assertEquals(rootPsd, estimator.getRootPsd(), ABSOLUTE_ERROR);

        // reset
        assertTrue(estimator.reset());

        assertEquals(mReset, 1);

        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertEquals(0.0, estimator.getAvg(), 0.0);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Override
    public void onStart(final AccumulatedAccelerationMeasurementNoiseEstimator estimator) {
        checkLocked(estimator);
        mStart++;
    }

    @Override
    public void onMeasurementAdded(
            final AccumulatedAccelerationMeasurementNoiseEstimator estimator) {
        mMeasurementAdded++;
    }

    @Override
    public void onReset(final AccumulatedAccelerationMeasurementNoiseEstimator estimator) {
        mReset++;
    }

    private void reset() {
        mStart = 0;
        mMeasurementAdded = 0;
        mReset = 0;
    }

    private void checkLocked(final AccumulatedAccelerationMeasurementNoiseEstimator estimator) {
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
            estimator.addMeasurement(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        final Acceleration a = new Acceleration(0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
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
