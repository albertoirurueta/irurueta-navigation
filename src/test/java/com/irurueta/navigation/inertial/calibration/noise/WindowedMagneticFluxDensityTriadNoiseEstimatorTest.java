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
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class WindowedMagneticFluxDensityTriadNoiseEstimatorTest implements
        WindowedMagneticFluxDensityTriadNoiseEstimatorListener {

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
    private int mTriadAdded;
    private int mWindowFilled;
    private int mReset;

    @Test
    public void testConstructor1() {
        final WindowedMagneticFluxDensityTriadNoiseEstimator estimator =
                new WindowedMagneticFluxDensityTriadNoiseEstimator();

        // check default values
        assertEquals(WindowedMagneticFluxDensityTriadNoiseEstimator.DEFAULT_WINDOW_SIZE,
                estimator.getWindowSize());
        assertEquals(WindowedMagneticFluxDensityTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedMagneticFluxDensityTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final MagneticFluxDensity avgX1 = estimator.getAvgXAsMeasurement();
        assertEquals(0.0, avgX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgX1.getUnit());
        final MagneticFluxDensity avgX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgXAsMeasurement(avgX2);
        assertEquals(avgX1, avgX2);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        final MagneticFluxDensity avgY1 = estimator.getAvgYAsMeasurement();
        assertEquals(0.0, avgY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgY1.getUnit());
        final MagneticFluxDensity avgY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgYAsMeasurement(avgY2);
        assertEquals(avgY1, avgY2);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        final MagneticFluxDensity avgZ1 = estimator.getAvgZAsMeasurement();
        assertEquals(0.0, avgZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgZ1.getUnit());
        final MagneticFluxDensity avgZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgZAsMeasurement(avgZ2);
        assertEquals(avgZ1, avgZ2);
        final MagneticFluxDensityTriad triad1 = estimator.getAvgTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        final MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad();
        estimator.getAvgTriad(triad2);
        assertEquals(triad1, triad2);
        assertEquals(0.0, estimator.getAvgNorm(), 0.0);
        final MagneticFluxDensity norm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(0.0, norm1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, norm1.getUnit());
        final MagneticFluxDensity norm2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgNormAsMeasurement(norm2);
        assertEquals(norm1, norm2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getStandardDeviationNorm(), 0.0);
        final MagneticFluxDensity stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(0.0, stdNorm1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdNorm1.getUnit());
        final MagneticFluxDensity stdNorm2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final MagneticFluxDensity avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
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
        final WindowedMagneticFluxDensityTriadNoiseEstimator estimator =
                new WindowedMagneticFluxDensityTriadNoiseEstimator(this);

        // check default values
        assertEquals(WindowedMagneticFluxDensityTriadNoiseEstimator.DEFAULT_WINDOW_SIZE,
                estimator.getWindowSize());
        assertEquals(WindowedMagneticFluxDensityTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedMagneticFluxDensityTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final MagneticFluxDensity avgX1 = estimator.getAvgXAsMeasurement();
        assertEquals(0.0, avgX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgX1.getUnit());
        final MagneticFluxDensity avgX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgXAsMeasurement(avgX2);
        assertEquals(avgX1, avgX2);
        assertEquals(0.0, estimator.getAvgY(), 0.0);
        final MagneticFluxDensity avgY1 = estimator.getAvgYAsMeasurement();
        assertEquals(0.0, avgY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgY1.getUnit());
        final MagneticFluxDensity avgY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgYAsMeasurement(avgY2);
        assertEquals(avgY1, avgY2);
        assertEquals(0.0, estimator.getAvgZ(), 0.0);
        final MagneticFluxDensity avgZ1 = estimator.getAvgZAsMeasurement();
        assertEquals(0.0, avgZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgZ1.getUnit());
        final MagneticFluxDensity avgZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgZAsMeasurement(avgZ2);
        assertEquals(avgZ1, avgZ2);
        final MagneticFluxDensityTriad triad1 = estimator.getAvgTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        final MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad();
        estimator.getAvgTriad(triad2);
        assertEquals(triad1, triad2);
        assertEquals(0.0, estimator.getAvgNorm(), 0.0);
        final MagneticFluxDensity norm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(0.0, norm1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, norm1.getUnit());
        final MagneticFluxDensity norm2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgNormAsMeasurement(norm2);
        assertEquals(norm1, norm2);
        assertEquals(0.0, estimator.getVarianceX(), 0.0);
        assertEquals(0.0, estimator.getVarianceY(), 0.0);
        assertEquals(0.0, estimator.getVarianceZ(), 0.0);
        assertEquals(0.0, estimator.getStandardDeviationX(), 0.0);
        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(0.0, stdX1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        assertEquals(0.0, estimator.getStandardDeviationY(), 0.0);
        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(0.0, stdY1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        assertEquals(0.0, estimator.getStandardDeviationZ(), 0.0);
        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(0.0, stdZ1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);
        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(0.0, stdTriad1.getValueX(), 0.0);
        assertEquals(0.0, stdTriad1.getValueY(), 0.0);
        assertEquals(0.0, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
        assertEquals(0.0, estimator.getStandardDeviationNorm(), 0.0);
        final MagneticFluxDensity stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(0.0, stdNorm1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdNorm1.getUnit());
        final MagneticFluxDensity stdNorm2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);
        assertEquals(0.0, estimator.getAverageStandardDeviation(), 0.0);
        final MagneticFluxDensity avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(0.0, avgStd1.getValue().doubleValue(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
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
        final WindowedMagneticFluxDensityTriadNoiseEstimator estimator =
                new WindowedMagneticFluxDensityTriadNoiseEstimator();

        // check default value
        assertEquals(WindowedMagneticFluxDensityTriadNoiseEstimator.DEFAULT_WINDOW_SIZE,
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
        final WindowedMagneticFluxDensityTriadNoiseEstimator estimator =
                new WindowedMagneticFluxDensityTriadNoiseEstimator();

        // check default value
        assertEquals(WindowedMagneticFluxDensityTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final WindowedMagneticFluxDensityTriadNoiseEstimator estimator =
                new WindowedMagneticFluxDensityTriadNoiseEstimator();

        // check default value
        final Time time1 = estimator.getTimeIntervalAsTime();
        assertEquals(WindowedMagneticFluxDensityTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
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
        final WindowedMagneticFluxDensityTriadNoiseEstimator estimator =
                new WindowedMagneticFluxDensityTriadNoiseEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testAddTriadAndProcessAndThenReset1() throws LockedException, IOException {
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

        final WindowedMagneticFluxDensityTriadNoiseEstimator estimator =
                new WindowedMagneticFluxDensityTriadNoiseEstimator(this);

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

        final int windowSize = estimator.getWindowSize();
        final double timeInterval = estimator.getTimeInterval();
        final MagneticFluxDensityTriad firstTriad = new MagneticFluxDensityTriad();
        final MagneticFluxDensityTriad firstTriad2 = new MagneticFluxDensityTriad();
        final MagneticFluxDensityTriad lastTriad = new MagneticFluxDensityTriad();
        MagneticFluxDensityTriad triad = null;
        double valueX;
        double valueY;
        double valueZ;
        double avgBx = 0.0;
        double avgBy = 0.0;
        double avgBz = 0.0;
        final List<MagneticFluxDensityTriad> triads = new ArrayList<>();
        for (int i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedTriad(firstTriad)) {
                assertEquals(estimator.getFirstWindowedTriad(), firstTriad);
            }
            if (estimator.getLastWindowedTriad(lastTriad)) {
                assertEquals(estimator.getLastWindowedTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, position, cnb);

            if (i == 0) {
                firstTriad2.copyFrom(triad);
            }

            triads.add(new MagneticFluxDensityTriad(triad));
            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            assertEquals(estimator.addTriadAndProcess(triad), i != 0);

            assertTrue(estimator.getLastWindowedTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avgBx += valueX;
            avgBy += valueY;
            avgBz += valueZ;
        }

        avgBx /= windowSize;
        avgBy /= windowSize;
        avgBz /= windowSize;

        double varBx = 0.0;
        double varBy = 0.0;
        double varBz = 0.0;
        for (int i = 0; i < windowSize; i++) {
            triad.copyFrom(triads.get(i));

            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            final double diffX = valueX - avgBx;
            final double diffY = valueY - avgBy;
            final double diffZ = valueZ - avgBz;

            varBx += diffX * diffX;
            varBy += diffY * diffY;
            varBz += diffZ * diffZ;
        }

        varBx /= (windowSize - 1);
        varBy /= (windowSize - 1);
        varBz /= (windowSize - 1);

        final double stdBx = Math.sqrt(varBx);
        final double stdBy = Math.sqrt(varBy);
        final double stdBz = Math.sqrt(varBz);

        assertEquals(avgBx, estimator.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(avgBy, estimator.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(avgBz, estimator.getAvgZ(), ABSOLUTE_ERROR);

        MagneticFluxDensity b1 = estimator.getAvgXAsMeasurement();
        assertEquals(avgBx, b1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final MagneticFluxDensity b2 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgXAsMeasurement(b2);
        assertEquals(b1, b2);

        b1 = estimator.getAvgYAsMeasurement();
        assertEquals(avgBy, b1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        estimator.getAvgYAsMeasurement(b2);
        assertEquals(b1, b2);

        b1 = estimator.getAvgZAsMeasurement();
        assertEquals(avgBz, b1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        estimator.getAvgZAsMeasurement(b2);
        assertEquals(b1, b2);

        final MagneticFluxDensityTriad triad1 = estimator.getAvgTriad();
        assertEquals(avgBx, triad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(avgBy, triad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(avgBz, triad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        final MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad();
        estimator.getAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(triad1.getNorm(), estimator.getAvgNorm(), ABSOLUTE_ERROR);
        final MagneticFluxDensity norm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(triad1.getNorm(), norm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, norm1.getUnit());
        final MagneticFluxDensity norm2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgNormAsMeasurement(norm2);
        assertEquals(norm1, norm2);

        assertEquals(varBx, estimator.getVarianceX(), ABSOLUTE_ERROR);
        assertEquals(varBy, estimator.getVarianceY(), ABSOLUTE_ERROR);
        assertEquals(varBz, estimator.getVarianceZ(), ABSOLUTE_ERROR);

        assertEquals(stdBx, estimator.getStandardDeviationX(), ABSOLUTE_ERROR);
        assertEquals(stdBy, estimator.getStandardDeviationY(), ABSOLUTE_ERROR);
        assertEquals(stdBz, estimator.getStandardDeviationZ(), ABSOLUTE_ERROR);

        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(stdBx, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);

        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(stdBy, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);

        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(stdBz, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(stdBx, stdTriad1.getValueX(), 0.0);
        assertEquals(stdBy, stdTriad1.getValueY(), 0.0);
        assertEquals(stdBz, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);

        assertEquals(stdTriad1.getNorm(), estimator.getStandardDeviationNorm(), ABSOLUTE_ERROR);
        final MagneticFluxDensity stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(stdTriad1.getNorm(), stdNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdNorm1.getUnit());
        final MagneticFluxDensity stdNorm2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);

        final double avgStd = (stdBx + stdBy + stdBz) / 3.0;
        assertEquals(avgStd, estimator.getAverageStandardDeviation(), ABSOLUTE_ERROR);
        final MagneticFluxDensity avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(avgStd, avgStd1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);

        final double psdX = varBx * timeInterval;
        final double psdY = varBy * timeInterval;
        final double psdZ = varBz * timeInterval;

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
        triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                noiseRandomizer, timestamp, position, cnb);

        triads.add(new MagneticFluxDensityTriad(triad));

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
    public void testAddTriadAndProcessAndThenReset2() throws LockedException, IOException {
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

        final WindowedMagneticFluxDensityTriadNoiseEstimator estimator =
                new WindowedMagneticFluxDensityTriadNoiseEstimator(this);

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

        final int windowSize = estimator.getWindowSize();
        final double timeInterval = estimator.getTimeInterval();
        final MagneticFluxDensityTriad firstTriad = new MagneticFluxDensityTriad();
        final MagneticFluxDensityTriad firstTriad2 = new MagneticFluxDensityTriad();
        final MagneticFluxDensityTriad lastTriad = new MagneticFluxDensityTriad();
        MagneticFluxDensityTriad triad = null;
        double valueX;
        double valueY;
        double valueZ;
        double avgBx = 0.0;
        double avgBy = 0.0;
        double avgBz = 0.0;
        final List<MagneticFluxDensityTriad> triads = new ArrayList<>();
        for (int i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedTriad(firstTriad)) {
                assertEquals(estimator.getFirstWindowedTriad(), firstTriad);
                assertEquals(firstTriad, firstTriad);
            }
            if (estimator.getLastWindowedTriad(lastTriad)) {
                assertEquals(estimator.getLastWindowedTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, position, cnb);

            if (i == 0) {
                firstTriad2.copyFrom(triad);
            }

            triads.add(new MagneticFluxDensityTriad(triad));
            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            assertEquals(estimator.addTriadAndProcess(valueX, valueY, valueZ), i != 0);

            assertTrue(estimator.getLastWindowedTriad(lastTriad));
            assertEquals(lastTriad, triad);
            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());

            avgBx += valueX;
            avgBy += valueY;
            avgBz += valueZ;
        }

        avgBx /= windowSize;
        avgBy /= windowSize;
        avgBz /= windowSize;

        double varBx = 0.0;
        double varBy = 0.0;
        double varBz = 0.0;
        for (int i = 0; i < windowSize; i++) {
            triad.copyFrom(triads.get(i));

            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            final double diffX = valueX - avgBx;
            final double diffY = valueY - avgBy;
            final double diffZ = valueZ - avgBz;

            varBx += diffX * diffX;
            varBy += diffY * diffY;
            varBz += diffZ * diffZ;
        }

        varBx /= (windowSize - 1);
        varBy /= (windowSize - 1);
        varBz /= (windowSize - 1);

        final double stdBx = Math.sqrt(varBx);
        final double stdBy = Math.sqrt(varBy);
        final double stdBz = Math.sqrt(varBz);

        assertEquals(avgBx, estimator.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(avgBy, estimator.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(avgBz, estimator.getAvgZ(), ABSOLUTE_ERROR);

        MagneticFluxDensity b1 = estimator.getAvgXAsMeasurement();
        assertEquals(avgBx, b1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final MagneticFluxDensity b2 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgXAsMeasurement(b2);
        assertEquals(b1, b2);

        b1 = estimator.getAvgYAsMeasurement();
        assertEquals(avgBy, b1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        estimator.getAvgYAsMeasurement(b2);
        assertEquals(b1, b2);

        b1 = estimator.getAvgZAsMeasurement();
        assertEquals(avgBz, b1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        estimator.getAvgZAsMeasurement(b2);
        assertEquals(b1, b2);

        final MagneticFluxDensityTriad triad1 = estimator.getAvgTriad();
        assertEquals(avgBx, triad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(avgBy, triad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(avgBz, triad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        final MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad();
        estimator.getAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(triad1.getNorm(), estimator.getAvgNorm(), ABSOLUTE_ERROR);
        final MagneticFluxDensity norm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(triad1.getNorm(), norm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, norm1.getUnit());
        final MagneticFluxDensity norm2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgNormAsMeasurement(norm2);
        assertEquals(norm1, norm2);

        assertEquals(varBx, estimator.getVarianceX(), ABSOLUTE_ERROR);
        assertEquals(varBy, estimator.getVarianceY(), ABSOLUTE_ERROR);
        assertEquals(varBz, estimator.getVarianceZ(), ABSOLUTE_ERROR);

        assertEquals(stdBx, estimator.getStandardDeviationX(), ABSOLUTE_ERROR);
        assertEquals(stdBy, estimator.getStandardDeviationY(), ABSOLUTE_ERROR);
        assertEquals(stdBz, estimator.getStandardDeviationZ(), ABSOLUTE_ERROR);

        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(stdBx, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);

        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(stdBy, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);

        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(stdBz, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(stdBx, stdTriad1.getValueX(), 0.0);
        assertEquals(stdBy, stdTriad1.getValueY(), 0.0);
        assertEquals(stdBz, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);

        assertEquals(stdTriad1.getNorm(), estimator.getStandardDeviationNorm(), ABSOLUTE_ERROR);
        final MagneticFluxDensity stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(stdTriad1.getNorm(), stdNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdNorm1.getUnit());
        final MagneticFluxDensity stdNorm2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);

        final double avgStd = (stdBx + stdBy + stdBz) / 3.0;
        assertEquals(avgStd, estimator.getAverageStandardDeviation(), ABSOLUTE_ERROR);
        final MagneticFluxDensity avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(avgStd, avgStd1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);

        final double psdX = varBx * timeInterval;
        final double psdY = varBy * timeInterval;
        final double psdZ = varBz * timeInterval;

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
        triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                noiseRandomizer, timestamp, position, cnb);

        triads.add(new MagneticFluxDensityTriad(triad));

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
    public void testAddTriadAndProcessAndThenReset3() throws LockedException, IOException {
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

        final WindowedMagneticFluxDensityTriadNoiseEstimator estimator =
                new WindowedMagneticFluxDensityTriadNoiseEstimator(this);

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

        final int windowSize = estimator.getWindowSize();
        final double timeInterval = estimator.getTimeInterval();
        final MagneticFluxDensityTriad firstTriad = new MagneticFluxDensityTriad();
        final MagneticFluxDensityTriad firstTriad2 = new MagneticFluxDensityTriad();
        final MagneticFluxDensityTriad lastTriad = new MagneticFluxDensityTriad();
        MagneticFluxDensityTriad triad = null;
        double valueX;
        double valueY;
        double valueZ;
        double avgBx = 0.0;
        double avgBy = 0.0;
        double avgBz = 0.0;
        final List<MagneticFluxDensityTriad> triads = new ArrayList<>();
        for (int i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedTriad(firstTriad)) {
                assertEquals(estimator.getFirstWindowedTriad(), firstTriad);
                assertEquals(firstTriad, firstTriad);
            }
            if (estimator.getLastWindowedTriad(lastTriad)) {
                assertEquals(estimator.getLastWindowedTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, position, cnb);

            if (i == 0) {
                firstTriad2.copyFrom(triad);
            }

            triads.add(new MagneticFluxDensityTriad(triad));
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

            avgBx += valueX;
            avgBy += valueY;
            avgBz += valueZ;
        }

        avgBx /= windowSize;
        avgBy /= windowSize;
        avgBz /= windowSize;

        double varBx = 0.0;
        double varBy = 0.0;
        double varBz = 0.0;
        for (int i = 0; i < windowSize; i++) {
            triad.copyFrom(triads.get(i));

            valueX = triad.getValueX();
            valueY = triad.getValueY();
            valueZ = triad.getValueZ();

            final double diffX = valueX - avgBx;
            final double diffY = valueY - avgBy;
            final double diffZ = valueZ - avgBz;

            varBx += diffX * diffX;
            varBy += diffY * diffY;
            varBz += diffZ * diffZ;
        }

        varBx /= (windowSize - 1);
        varBy /= (windowSize - 1);
        varBz /= (windowSize - 1);

        final double stdBx = Math.sqrt(varBx);
        final double stdBy = Math.sqrt(varBy);
        final double stdBz = Math.sqrt(varBz);

        assertEquals(avgBx, estimator.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(avgBy, estimator.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(avgBz, estimator.getAvgZ(), ABSOLUTE_ERROR);

        MagneticFluxDensity b1 = estimator.getAvgXAsMeasurement();
        assertEquals(avgBx, b1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        final MagneticFluxDensity b2 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgXAsMeasurement(b2);
        assertEquals(b1, b2);

        b1 = estimator.getAvgYAsMeasurement();
        assertEquals(avgBy, b1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        estimator.getAvgYAsMeasurement(b2);
        assertEquals(b1, b2);

        b1 = estimator.getAvgZAsMeasurement();
        assertEquals(avgBz, b1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());
        estimator.getAvgZAsMeasurement(b2);
        assertEquals(b1, b2);

        final MagneticFluxDensityTriad triad1 = estimator.getAvgTriad();
        assertEquals(avgBx, triad1.getValueX(), ABSOLUTE_ERROR);
        assertEquals(avgBy, triad1.getValueY(), ABSOLUTE_ERROR);
        assertEquals(avgBz, triad1.getValueZ(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, triad1.getUnit());
        final MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad();
        estimator.getAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(triad1.getNorm(), estimator.getAvgNorm(), ABSOLUTE_ERROR);
        final MagneticFluxDensity norm1 = estimator.getAvgNormAsMeasurement();
        assertEquals(triad1.getNorm(), norm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, norm1.getUnit());
        final MagneticFluxDensity norm2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAvgNormAsMeasurement(norm2);
        assertEquals(norm1, norm2);

        assertEquals(varBx, estimator.getVarianceX(), ABSOLUTE_ERROR);
        assertEquals(varBy, estimator.getVarianceY(), ABSOLUTE_ERROR);
        assertEquals(varBz, estimator.getVarianceZ(), ABSOLUTE_ERROR);

        assertEquals(stdBx, estimator.getStandardDeviationX(), ABSOLUTE_ERROR);
        assertEquals(stdBy, estimator.getStandardDeviationY(), ABSOLUTE_ERROR);
        assertEquals(stdBz, estimator.getStandardDeviationZ(), ABSOLUTE_ERROR);

        final MagneticFluxDensity stdX1 = estimator.getStandardDeviationXAsMeasurement();
        assertEquals(stdBx, stdX1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdX1.getUnit());
        final MagneticFluxDensity stdX2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);

        final MagneticFluxDensity stdY1 = estimator.getStandardDeviationYAsMeasurement();
        assertEquals(stdBy, stdY1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdY1.getUnit());
        final MagneticFluxDensity stdY2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);

        final MagneticFluxDensity stdZ1 = estimator.getStandardDeviationZAsMeasurement();
        assertEquals(stdBz, stdZ1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdZ1.getUnit());
        final MagneticFluxDensity stdZ2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        final MagneticFluxDensityTriad stdTriad1 = estimator.getStandardDeviationTriad();
        assertEquals(stdBx, stdTriad1.getValueX(), 0.0);
        assertEquals(stdBy, stdTriad1.getValueY(), 0.0);
        assertEquals(stdBz, stdTriad1.getValueZ(), 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdTriad1.getUnit());
        final MagneticFluxDensityTriad stdTriad2 = new MagneticFluxDensityTriad();
        estimator.getStandardDeviationTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);

        assertEquals(stdTriad1.getNorm(), estimator.getStandardDeviationNorm(), ABSOLUTE_ERROR);
        final MagneticFluxDensity stdNorm1 = estimator.getStandardDeviationNormAsMeasurement();
        assertEquals(stdTriad1.getNorm(), stdNorm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, stdNorm1.getUnit());
        final MagneticFluxDensity stdNorm2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getStandardDeviationNormAsMeasurement(stdNorm2);
        assertEquals(stdNorm1, stdNorm2);

        final double avgStd = (stdBx + stdBy + stdBz) / 3.0;
        assertEquals(avgStd, estimator.getAverageStandardDeviation(), ABSOLUTE_ERROR);
        final MagneticFluxDensity avgStd1 = estimator.getAverageStandardDeviationAsMeasurement();
        assertEquals(avgStd, avgStd1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(MagneticFluxDensityUnit.TESLA, avgStd1.getUnit());
        final MagneticFluxDensity avgStd2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.NANOTESLA);
        estimator.getAverageStandardDeviationAsMeasurement(avgStd2);
        assertEquals(avgStd1, avgStd2);

        final double psdX = varBx * timeInterval;
        final double psdY = varBy * timeInterval;
        final double psdZ = varBz * timeInterval;

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
        triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                noiseRandomizer, timestamp, position, cnb);

        triads.add(new MagneticFluxDensityTriad(triad));

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
    public void testAddTriad1() throws LockedException, IOException {
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

        final WindowedMagneticFluxDensityTriadNoiseEstimator estimator =
                new WindowedMagneticFluxDensityTriadNoiseEstimator(this);

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

        final int windowSize = estimator.getWindowSize();
        final MagneticFluxDensityTriad firstTriad = new MagneticFluxDensityTriad();
        final MagneticFluxDensityTriad firstTriad2 = new MagneticFluxDensityTriad();
        final MagneticFluxDensityTriad lastTriad = new MagneticFluxDensityTriad();
        MagneticFluxDensityTriad triad = null;
        final List<MagneticFluxDensityTriad> triads = new ArrayList<>();
        for (int i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedTriad(firstTriad)) {
                assertEquals(estimator.getFirstWindowedTriad(), firstTriad);
                assertEquals(firstTriad, firstTriad);
            }
            if (estimator.getLastWindowedTriad(lastTriad)) {
                assertEquals(estimator.getLastWindowedTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, position, cnb);

            if (i == 0) {
                firstTriad2.copyFrom(triad);
            }

            triads.add(new MagneticFluxDensityTriad(triad));
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
    public void testAddTriad2() throws LockedException, IOException {
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

        final WindowedMagneticFluxDensityTriadNoiseEstimator estimator =
                new WindowedMagneticFluxDensityTriadNoiseEstimator(this);

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

        final int windowSize = estimator.getWindowSize();
        final MagneticFluxDensityTriad firstTriad = new MagneticFluxDensityTriad();
        final MagneticFluxDensityTriad firstTriad2 = new MagneticFluxDensityTriad();
        final MagneticFluxDensityTriad lastTriad = new MagneticFluxDensityTriad();
        MagneticFluxDensityTriad triad = null;
        final List<MagneticFluxDensityTriad> triads = new ArrayList<>();
        for (int i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedTriad(firstTriad)) {
                assertEquals(estimator.getFirstWindowedTriad(), firstTriad);
                assertEquals(firstTriad, firstTriad);
            }
            if (estimator.getLastWindowedTriad(lastTriad)) {
                assertEquals(estimator.getLastWindowedTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, position, cnb);

            if (i == 0) {
                firstTriad2.copyFrom(triad);
            }

            triads.add(new MagneticFluxDensityTriad(triad));
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
    public void testAddTriad3() throws LockedException, IOException {
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

        final WindowedMagneticFluxDensityTriadNoiseEstimator estimator =
                new WindowedMagneticFluxDensityTriadNoiseEstimator(this);

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

        final int windowSize = estimator.getWindowSize();
        final MagneticFluxDensityTriad firstTriad = new MagneticFluxDensityTriad();
        final MagneticFluxDensityTriad firstTriad2 = new MagneticFluxDensityTriad();
        final MagneticFluxDensityTriad lastTriad = new MagneticFluxDensityTriad();
        MagneticFluxDensityTriad triad = null;
        final List<MagneticFluxDensityTriad> triads = new ArrayList<>();
        for (int i = 0; i < windowSize; i++) {
            if (estimator.getFirstWindowedTriad(firstTriad)) {
                assertEquals(estimator.getFirstWindowedTriad(), firstTriad);
                assertEquals(firstTriad, firstTriad);
            }
            if (estimator.getLastWindowedTriad(lastTriad)) {
                assertEquals(estimator.getLastWindowedTriad(), lastTriad);
                assertEquals(lastTriad, triad);
            }

            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, position, cnb);

            if (i == 0) {
                firstTriad2.copyFrom(triad);
            }

            triads.add(new MagneticFluxDensityTriad(triad));
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
    public void onStart(final WindowedMagneticFluxDensityTriadNoiseEstimator estimator) {
        checkLocked(estimator);
        mStart++;
    }

    @Override
    public void onTriadAdded(final WindowedMagneticFluxDensityTriadNoiseEstimator estimator) {
        mTriadAdded++;
    }

    @Override
    public void onWindowFilled(final WindowedMagneticFluxDensityTriadNoiseEstimator estimator) {
        mWindowFilled++;
    }

    @Override
    public void onReset(final WindowedMagneticFluxDensityTriadNoiseEstimator estimator) {
        mReset++;
    }

    private void reset() {
        mStart = 0;
        mTriadAdded = 0;
        mWindowFilled = 0;
        mReset = 0;
    }

    private void checkLocked(final WindowedMagneticFluxDensityTriadNoiseEstimator estimator) {
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
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0,
                MagneticFluxDensityUnit.TESLA);
        try {
            estimator.addTriadAndProcess(b, b, b);
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
            estimator.addTriad(b, b, b);
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
