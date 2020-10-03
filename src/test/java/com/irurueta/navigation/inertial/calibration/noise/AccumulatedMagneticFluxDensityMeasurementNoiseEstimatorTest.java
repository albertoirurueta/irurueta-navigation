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
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.geodesic.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.calibration.BodyMagneticFluxDensityGenerator;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.io.IOException;
import java.util.Calendar;
import java.util.Date;
import java.util.Random;

import static org.junit.Assert.*;

public class AccumulatedMagneticFluxDensityMeasurementNoiseEstimatorTest implements
        AccumulatedMagneticFluxDensityMeasurementNoiseEstimatorListener {

    private static final double MIN_HARD_IRON = -1e-5;
    private static final double MAX_HARD_IRON = 1e-5;

    private static final double MIN_SOFT_IRON = -1e-6;
    private static final double MAX_SOFT_IRON = 1e-6;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT_METERS = -500.0;
    private static final double MAX_HEIGHT_METERS = 10000.0;

    private static final double MAGNETOMETER_NOISE_STD = 200e-9;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final int N_SAMPLES = 1000;

    private static final Calendar START_CALENDAR = Calendar.getInstance();
    private static final Calendar END_CALENDAR = Calendar.getInstance();

    private static final long START_TIMESTAMP_MILLIS;
    private static final long END_TIMESTAMP_MILLIS;

    static {
        START_CALENDAR.set(2020, Calendar.JANUARY, 1,
                0, 0, 0);
        END_CALENDAR.set(2025, Calendar.DECEMBER, 31,
                23, 59, 59);

        START_TIMESTAMP_MILLIS = START_CALENDAR.getTimeInMillis();
        END_TIMESTAMP_MILLIS = END_CALENDAR.getTimeInMillis();
    }

    private int mStart;
    private int mMeasurementAdded;
    private int mReset;

    @Test
    public void testConstructor1() {
        final AccumulatedMagneticFluxDensityMeasurementNoiseEstimator estimator =
                new AccumulatedMagneticFluxDensityMeasurementNoiseEstimator();

        // check default values
        assertEquals(AccumulatedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertNull(estimator.getListener());
        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertEquals(0.0, estimator.getAvg(), 0.0);
        final MagneticFluxDensity avg1 = estimator.getAvgAsMeasurement();
        assertEquals(0.0, avg1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avg1.getUnit());
        final MagneticFluxDensity avg2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        final MagneticFluxDensity std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(0.0, std1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, std1.getUnit());
        final MagneticFluxDensity std2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Test
    public void testConstructor2() {
        final AccumulatedMagneticFluxDensityMeasurementNoiseEstimator estimator =
                new AccumulatedMagneticFluxDensityMeasurementNoiseEstimator(this);

        // check default values
        assertEquals(AccumulatedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(AccumulatedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                time1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, time1.getUnit());
        final Time time2 = new Time(0.0, TimeUnit.HOUR);
        estimator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(this, estimator.getListener());
        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertEquals(0.0, estimator.getAvg(), 0.0);
        final MagneticFluxDensity avg1 = estimator.getAvgAsMeasurement();
        assertEquals(0.0, avg1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avg1.getUnit());
        final MagneticFluxDensity avg2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);
        assertEquals(0.0, estimator.getVariance(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviation(), 0.0);
        final MagneticFluxDensity std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(0.0, std1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, std1.getUnit());
        final MagneticFluxDensity std2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationAsMeasurement(std2);
        assertEquals(std1, std2);
        assertEquals(0.0, estimator.getPsd(), 0.0);
        assertEquals(0.0, estimator.getRootPsd(), 0.0);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
    }

    @Test
    public void testGetSetTimeInterval() throws LockedException {
        final AccumulatedMagneticFluxDensityMeasurementNoiseEstimator estimator =
                new AccumulatedMagneticFluxDensityMeasurementNoiseEstimator();

        // check default value
        assertEquals(
                AccumulatedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final AccumulatedMagneticFluxDensityMeasurementNoiseEstimator estimator =
                new AccumulatedMagneticFluxDensityMeasurementNoiseEstimator();

        // check default value
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(
                AccumulatedMagneticFluxDensityMeasurementNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final AccumulatedMagneticFluxDensityMeasurementNoiseEstimator estimator =
                new AccumulatedMagneticFluxDensityMeasurementNoiseEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testAddTriadAndReset1() throws LockedException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, MAGNETOMETER_NOISE_STD);

        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));

        final AccumulatedMagneticFluxDensityMeasurementNoiseEstimator estimator =
                new AccumulatedMagneticFluxDensityMeasurementNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mMeasurementAdded, 0);
        assertEquals(mReset, 0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertFalse(estimator.isRunning());

        MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();
        final double timeInterval = estimator.getTimeInterval();
        final MagneticFluxDensity lastMeasurement = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        double value;
        double avg = 0.0;
        double var = 0.0;
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastMeasurement(lastMeasurement)) {
                assertEquals(estimator.getLastMeasurement(), lastMeasurement);
                assertEquals(lastMeasurement, triad.getMeasurementNorm());
            }

            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, position, cnb);

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

        final MagneticFluxDensity avg1 = estimator.getAvgAsMeasurement();
        assertEquals(avg, avg1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avg1.getUnit());
        final MagneticFluxDensity avg2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);

        assertEquals(var, estimator.getVariance(), ABSOLUTE_ERROR);

        final double std = Math.sqrt(var);

        assertEquals(std, estimator.getStandardDeviation(), ABSOLUTE_ERROR);

        final MagneticFluxDensity std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(std, std1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, std1.getUnit());
        final MagneticFluxDensity std2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
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
    public void testAddTriadAndReset2() throws LockedException, IOException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, MAGNETOMETER_NOISE_STD);

        final NEDPosition position = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));

        final AccumulatedMagneticFluxDensityMeasurementNoiseEstimator estimator =
                new AccumulatedMagneticFluxDensityMeasurementNoiseEstimator(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mMeasurementAdded, 0);
        assertEquals(mReset, 0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertNull(estimator.getLastMeasurement());
        assertFalse(estimator.getLastMeasurement(null));
        assertFalse(estimator.isRunning());

        MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();
        final double timeInterval = estimator.getTimeInterval();
        final MagneticFluxDensity lastMeasurement = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        double value;
        double avg = 0.0;
        double var = 0.0;
        for (int i = 0, j = 1; i < N_SAMPLES; i++, j++) {
            if (estimator.getLastMeasurement(lastMeasurement)) {
                assertEquals(estimator.getLastMeasurement(), lastMeasurement);
                assertEquals(lastMeasurement, triad.getMeasurementNorm());
            }

            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, position, cnb);

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

        final MagneticFluxDensity avg1 = estimator.getAvgAsMeasurement();
        assertEquals(avg, avg1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avg1.getUnit());
        final MagneticFluxDensity avg2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgAsMeasurement(avg2);
        assertEquals(avg1, avg2);

        assertEquals(var, estimator.getVariance(), ABSOLUTE_ERROR);

        final double std = Math.sqrt(var);

        assertEquals(std, estimator.getStandardDeviation(), ABSOLUTE_ERROR);

        final MagneticFluxDensity std1 = estimator.getStandardDeviationAsMeasurement();
        assertEquals(std, std1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, std1.getUnit());
        final MagneticFluxDensity std2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
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
    public void onStart(final AccumulatedMagneticFluxDensityMeasurementNoiseEstimator estimator) {
        checkLocked(estimator);
        mStart++;
    }

    @Override
    public void onMeasurementAdded(
            final AccumulatedMagneticFluxDensityMeasurementNoiseEstimator estimator) {
        mMeasurementAdded++;
    }

    @Override
    public void onReset(final AccumulatedMagneticFluxDensityMeasurementNoiseEstimator estimator) {
        mReset++;
    }

    private void reset() {
        mStart = 0;
        mMeasurementAdded = 0;
        mReset = 0;
    }

    private void checkLocked(final AccumulatedMagneticFluxDensityMeasurementNoiseEstimator estimator) {
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
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0,
                MagneticFluxDensityUnit.TESLA);
        try {
            estimator.addMeasurement(b);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.reset();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
    }

    private static MagneticFluxDensityTriad generateTriad(
            final double[] hardIron, final Matrix softIron,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final GaussianRandomizer noiseRandomizer,
            final Date timestamp,
            final NEDPosition position,
            final CoordinateTransformation cnb) {

        final NEDMagneticFluxDensity earthB = wmmEstimator.estimate(
                position, timestamp);

        final BodyMagneticFluxDensity truthMagnetic =
                BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final BodyMagneticFluxDensity measuredMagnetic =
                generateMeasuredMagneticFluxDensity(truthMagnetic,
                        hardIron, softIron);

        if (noiseRandomizer != null) {
            measuredMagnetic.setBx(measuredMagnetic.getBx()
                    + noiseRandomizer.nextDouble());
            measuredMagnetic.setBy(measuredMagnetic.getBy()
                    + noiseRandomizer.nextDouble());
            measuredMagnetic.setBz(measuredMagnetic.getBz()
                    + noiseRandomizer.nextDouble());
        }

        return measuredMagnetic.getCoordinatesAsTriad();
    }

    private static CoordinateTransformation generateBodyC(
            final UniformRandomizer randomizer) {

        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        return new CoordinateTransformation(
                roll, pitch, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
    }

    private static BodyMagneticFluxDensity generateMeasuredMagneticFluxDensity(
            final BodyMagneticFluxDensity input, final double[] hardIron,
            final Matrix softIron) {
        return BodyMagneticFluxDensityGenerator.generate(input, hardIron,
                softIron);
    }

    private static double[] generateHardIron(
            final UniformRandomizer randomizer) {
        final double[] result = new double[BodyMagneticFluxDensity.COMPONENTS];
        randomizer.fill(result, MIN_HARD_IRON, MAX_HARD_IRON);
        return result;
    }

    private static Matrix generateSoftIronGeneral() {
        try {
            return Matrix.createWithUniformRandomValues(
                    BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS, MIN_SOFT_IRON, MAX_SOFT_IRON);
        } catch (final WrongSizeException ignore) {
            // never happens
            return null;
        }
    }

    private static NEDPosition createPosition(
            final UniformRandomizer randomizer) {
        final double latitude = Math.toRadians(randomizer.nextDouble(
                MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(
                MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

        return new NEDPosition(latitude, longitude, height);
    }

    private static long createTimestamp(final UniformRandomizer randomizer) {
        return randomizer.nextLong(
                START_TIMESTAMP_MILLIS, END_TIMESTAMP_MILLIS);
    }
}
