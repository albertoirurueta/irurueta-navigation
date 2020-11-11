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
package com.irurueta.navigation.inertial.calibration.generators;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.geodesic.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.BodyKinematicsAndMagneticFluxDensity;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.BodyMagneticFluxDensityGenerator;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.intervals.AccelerationTriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.magnetometer.KnownPositionAndInstantMagnetometerCalibrator;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import org.junit.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class MagnetometerMeasurementGeneratorTest implements MagnetometerMeasurementsGeneratorListener {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

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

    private static final double SMALL_MAGNETOMETER_NOISE_STD = 1e-12;
    private static final double MAGNETOMETER_NOISE_STD = 200e-9;

    private static final double MIN_DELTA_POS_METERS = -0.01;
    private static final double MAX_DELTA_POS_METERS = 0.01;
    private static final double MIN_DELTA_ANGLE_DEGREES = -2.0;
    private static final double MAX_DELTA_ANGLE_DEGREES = 2.0;

    private static final int TIMES = 100;

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-5;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;

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

    private int mInitializationStarted;
    private int mInitializationCompleted;
    private int mError;
    private int mStaticIntervalDetected;
    private int mDynamicIntervalDetected;
    private int mStaticIntervalSkipped;
    private int mDynamicIntervalSkipped;
    private int mGeneratedMeasurement;
    private int mReset;

    private final List<StandardDeviationBodyMagneticFluxDensity> mMeasurements =
            new ArrayList<>();

    @Test
    public void testConstructor1() {
        final MagnetometerMeasurementsGenerator generator =
                new MagnetometerMeasurementsGenerator();

        // check default values
        assertEquals(generator.getMinStaticSamples(),
                MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES);
        assertEquals(generator.getMaxDynamicSamples(),
                MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES);
        assertNull(generator.getListener());
        assertEquals(generator.getProcessedStaticSamples(), 0);
        assertEquals(generator.getProcessedDynamicSamples(), 0);
        assertFalse(generator.isStaticIntervalSkipped());
        assertFalse(generator.isDynamicIntervalSkipped());
        assertFalse(generator.isRunning());

        assertEquals(generator.getWindowSize(),
                TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE);
        assertEquals(generator.getInitialStaticSamples(),
                TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES);
        assertEquals(generator.getThresholdFactor(),
                TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, 0.0);
        assertEquals(generator.getInstantaneousNoiseLevelFactor(),
                TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                0.0);
        assertEquals(generator.getBaseNoiseLevelAbsoluteThreshold(),
                TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                0.0);
        final Acceleration errorThreshold1 = generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(errorThreshold1.getValue().doubleValue(),
                generator.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        assertEquals(errorThreshold1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration errorThreshold2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(errorThreshold2);
        assertEquals(errorThreshold1, errorThreshold2);
        assertEquals(generator.getStatus(),
                TriadStaticIntervalDetector.Status.IDLE);
        assertEquals(generator.getBaseNoiseLevel(), 0.0, 0.0);
        final Acceleration baseNoiseLevel1 = generator.getBaseNoiseLevelAsMeasurement();
        assertEquals(baseNoiseLevel1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(baseNoiseLevel1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baseNoiseLevel2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getBaseNoiseLevelAsMeasurement(baseNoiseLevel2);
        assertEquals(baseNoiseLevel1, baseNoiseLevel2);
        assertEquals(generator.getThreshold(), 0.0, 0.0);
        final Acceleration threshold1 = generator.getThresholdAsMeasurement();
        assertEquals(threshold1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(threshold1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration threshold2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getThresholdAsMeasurement(threshold2);
        assertEquals(threshold1, threshold2);
    }

    @Test
    public void testConstructor2() {
        final MagnetometerMeasurementsGenerator generator =
                new MagnetometerMeasurementsGenerator(this);

        // check default values
        assertEquals(generator.getMinStaticSamples(),
                MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES);
        assertEquals(generator.getMaxDynamicSamples(),
                MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES);
        assertSame(generator.getListener(), this);
        assertEquals(generator.getProcessedStaticSamples(), 0);
        assertEquals(generator.getProcessedDynamicSamples(), 0);
        assertFalse(generator.isStaticIntervalSkipped());
        assertFalse(generator.isDynamicIntervalSkipped());
        assertFalse(generator.isRunning());

        assertEquals(generator.getWindowSize(),
                TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE);
        assertEquals(generator.getInitialStaticSamples(),
                TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES);
        assertEquals(generator.getThresholdFactor(),
                TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, 0.0);
        assertEquals(generator.getInstantaneousNoiseLevelFactor(),
                TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                0.0);
        assertEquals(generator.getBaseNoiseLevelAbsoluteThreshold(),
                TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                0.0);
        final Acceleration errorThreshold1 = generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(errorThreshold1.getValue().doubleValue(),
                generator.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        assertEquals(errorThreshold1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration errorThreshold2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(errorThreshold2);
        assertEquals(errorThreshold1, errorThreshold2);
        assertEquals(generator.getStatus(),
                TriadStaticIntervalDetector.Status.IDLE);
        assertEquals(generator.getBaseNoiseLevel(), 0.0, 0.0);
        final Acceleration baseNoiseLevel1 = generator.getBaseNoiseLevelAsMeasurement();
        assertEquals(baseNoiseLevel1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(baseNoiseLevel1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baseNoiseLevel2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getBaseNoiseLevelAsMeasurement(baseNoiseLevel2);
        assertEquals(baseNoiseLevel1, baseNoiseLevel2);
        assertEquals(generator.getThreshold(), 0.0, 0.0);
        final Acceleration threshold1 = generator.getThresholdAsMeasurement();
        assertEquals(threshold1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(threshold1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration threshold2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getThresholdAsMeasurement(threshold2);
        assertEquals(threshold1, threshold2);
    }

    @Test
    public void testGetSetMinStaticSamples() throws LockedException {
        final MagnetometerMeasurementsGenerator generator =
                new MagnetometerMeasurementsGenerator();

        // check default value
        assertEquals(generator.getMinStaticSamples(),
                MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES);

        // set new value
        generator.setMinStaticSamples(10);

        // check
        assertEquals(10, generator.getMinStaticSamples());

        // Force IllegalArgumentException
        try {
            generator.setMinStaticSamples(1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMaxDynamicSamples() throws LockedException {
        final MagnetometerMeasurementsGenerator generator =
                new MagnetometerMeasurementsGenerator();

        // check default value
        assertEquals(generator.getMaxDynamicSamples(),
                MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES);

        // set new value
        generator.setMaxDynamicSamples(10);

        // check
        assertEquals(10, generator.getMaxDynamicSamples());

        // Force IllegalArgumentException
        try {
            generator.setMaxDynamicSamples(1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final MagnetometerMeasurementsGenerator generator =
                new MagnetometerMeasurementsGenerator();

        // check default value
        assertNull(generator.getListener());

        // set new value
        generator.setListener(this);

        // check
        assertSame(generator.getListener(), this);
    }

    @Test
    public void testGetSetWindowSize() throws LockedException {
        final MagnetometerMeasurementsGenerator generator =
                new MagnetometerMeasurementsGenerator();

        // check default value
        assertEquals(generator.getWindowSize(),
                TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE);

        // set new value
        generator.setWindowSize(3);

        // check
        assertEquals(generator.getWindowSize(), 3);

        // Force IllegalArgumentException
        try {
            generator.setWindowSize(1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            generator.setWindowSize(2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetInitialStaticSamples() throws LockedException {
        final MagnetometerMeasurementsGenerator generator =
                new MagnetometerMeasurementsGenerator();

        // check default value
        assertEquals(generator.getInitialStaticSamples(),
                TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES);

        // set new value
        generator.setInitialStaticSamples(2);

        // check
        assertEquals(generator.getInitialStaticSamples(), 2);

        // Force IllegalArgumentException
        try {
            generator.setInitialStaticSamples(1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetThresholdFactor() throws LockedException {
        final MagnetometerMeasurementsGenerator generator =
                new MagnetometerMeasurementsGenerator();

        // check default value
        assertEquals(generator.getThresholdFactor(),
                TriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, 0.0);

        // set new value
        generator.setThresholdFactor(1.0);

        // check
        assertEquals(generator.getThresholdFactor(), 1.0, 0.0);

        // Force IllegalArgumentException
        try {
            generator.setThresholdFactor(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetInstantaneousNoiseLevelFactor() throws LockedException {
        final MagnetometerMeasurementsGenerator generator =
                new MagnetometerMeasurementsGenerator();

        // check default value
        assertEquals(generator.getInstantaneousNoiseLevelFactor(),
                TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                0.0);

        // set new value
        generator.setInstantaneousNoiseLevelFactor(1.0);

        // check
        assertEquals(generator.getInstantaneousNoiseLevelFactor(),
                1.0, 0.0);

        // Force IllegalArgumentException
        try {
            generator.setInstantaneousNoiseLevelFactor(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetBaseNoiseLevelAbsoluteThreshold()
            throws LockedException {
        final MagnetometerMeasurementsGenerator generator =
                new MagnetometerMeasurementsGenerator();

        // check default value
        assertEquals(generator.getBaseNoiseLevelAbsoluteThreshold(),
                TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                0.0);

        // set new value
        generator.setBaseNoiseLevelAbsoluteThreshold(1.0);

        // check
        assertEquals(generator.getBaseNoiseLevelAbsoluteThreshold(),
                1.0, 0.0);

        // Force IllegalArgumentException
        try {
            generator.setBaseNoiseLevelAbsoluteThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetBaseNoiseLevelAbsoluteThresholdAsMeasurement()
            throws LockedException {
        final MagnetometerMeasurementsGenerator generator =
                new MagnetometerMeasurementsGenerator();

        // check default value
        assertEquals(generator.getBaseNoiseLevelAbsoluteThreshold(),
                TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                0.0);

        final Acceleration a1 = generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(a1.getValue().doubleValue(),
                AccelerationTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // set new value
        final Acceleration a2 = new Acceleration(1.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        generator.setBaseNoiseLevelAbsoluteThreshold(a2);

        // check
        final Acceleration a3 = generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        final Acceleration a4 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(a4);
        assertEquals(a2, a3);
        assertEquals(a2, a4);
    }

    @Test
    public void testProcessCalibrateAndResetWithSmallNoiseAndCommonAxis() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            CalibrationException, NotReadyException, IOException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
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
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, SMALL_MAGNETOMETER_NOISE_STD);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final Date timestamp = new Date(createTimestamp(randomizer));
            NEDPosition nedPosition = createPosition(randomizer);
            CoordinateTransformation cnb = generateBodyC(randomizer);

            CoordinateTransformation nedC = cnb.inverseAndReturnNew();

            final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

            reset();
            assertTrue(mMeasurements.isEmpty());
            assertEquals(mInitializationStarted, 0);
            assertEquals(mInitializationCompleted, 0);
            assertEquals(mError, 0);
            assertEquals(mStaticIntervalDetected, 0);
            assertEquals(mDynamicIntervalDetected, 0);
            assertEquals(mGeneratedMeasurement, 0);
            assertEquals(mReset, 0);

            final MagnetometerMeasurementsGenerator generator =
                    new MagnetometerMeasurementsGenerator(this);

            // generate initial static samples
            final int initialStaticSamples = TriadStaticIntervalDetector
                    .DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics,
                    errors, hardIron, mm, wmmEstimator, random, timestamp, nedPosition,
                    cnb, noiseRandomizer);

            assertEquals(mInitializationStarted, 1);
            assertEquals(mInitializationCompleted, 1);

            final int numMeasurements = KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final int dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            for (int i = 0; i < numMeasurements; i++) {
                nedPosition = nedFrame.getPosition();
                nedC = nedFrame.getCoordinateTransformation();
                cnb = nedC.inverseAndReturnNew();

                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics,
                        errors, hardIron, mm, wmmEstimator, random, timestamp, nedPosition,
                        cnb, noiseRandomizer);

                assertEquals(mStaticIntervalDetected, i + 1);

                // generate dynamic samples
                generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics,
                        randomizer, ecefFrame, nedFrame, errors, hardIron, mm,
                        wmmEstimator, random, timestamp, nedPosition, cnb,
                        noiseRandomizer, false);

                assertEquals(mDynamicIntervalDetected, i + 1);
                assertEquals(mMeasurements.size(), i + 1);
            }

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(mReset, 1);
            assertEquals(mError, 0);

            KnownPositionAndInstantMagnetometerCalibrator calibrator =
                    new KnownPositionAndInstantMagnetometerCalibrator(
                            nedPosition, mMeasurements,
                            true);
            calibrator.setTime(timestamp);

            calibrator.calibrate();

            final Matrix estimatedHardIron = calibrator
                    .getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testProcessCalibrateAndResetWithSmallNoiseAndGeneralCase() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            CalibrationException, NotReadyException, IOException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
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
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, SMALL_MAGNETOMETER_NOISE_STD);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final Date timestamp = new Date(createTimestamp(randomizer));
            NEDPosition nedPosition = createPosition(randomizer);
            CoordinateTransformation cnb = generateBodyC(randomizer);

            CoordinateTransformation nedC = cnb.inverseAndReturnNew();

            final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

            reset();
            assertTrue(mMeasurements.isEmpty());
            assertEquals(mInitializationStarted, 0);
            assertEquals(mInitializationCompleted, 0);
            assertEquals(mError, 0);
            assertEquals(mStaticIntervalDetected, 0);
            assertEquals(mDynamicIntervalDetected, 0);
            assertEquals(mGeneratedMeasurement, 0);
            assertEquals(mReset, 0);

            final MagnetometerMeasurementsGenerator generator =
                    new MagnetometerMeasurementsGenerator(this);

            // generate initial static samples
            final int initialStaticSamples = TriadStaticIntervalDetector
                    .DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics,
                    errors, hardIron, mm, wmmEstimator, random, timestamp, nedPosition,
                    cnb, noiseRandomizer);

            assertEquals(mInitializationStarted, 1);
            assertEquals(mInitializationCompleted, 1);

            final int numMeasurements = KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final int dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            for (int i = 0; i < numMeasurements; i++) {
                nedPosition = nedFrame.getPosition();
                nedC = nedFrame.getCoordinateTransformation();
                cnb = nedC.inverseAndReturnNew();

                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics,
                        errors, hardIron, mm, wmmEstimator, random, timestamp, nedPosition,
                        cnb, noiseRandomizer);

                assertEquals(mStaticIntervalDetected, i + 1);

                // generate dynamic samples
                generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics,
                        randomizer, ecefFrame, nedFrame, errors, hardIron, mm,
                        wmmEstimator, random, timestamp, nedPosition, cnb,
                        noiseRandomizer, true);

                assertEquals(mDynamicIntervalDetected, i + 1);
                assertEquals(mMeasurements.size(), i + 1);
            }

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(mReset, 1);
            assertEquals(mError, 0);

            KnownPositionAndInstantMagnetometerCalibrator calibrator =
                    new KnownPositionAndInstantMagnetometerCalibrator(
                            nedPosition, mMeasurements,
                            false);
            calibrator.setTime(timestamp);

            calibrator.calibrate();

            final Matrix estimatedHardIron = calibrator
                    .getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testProcessCalibrateAndResetWithNoiseAndCommonAxis() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            CalibrationException, NotReadyException, IOException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
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
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, MAGNETOMETER_NOISE_STD);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final Date timestamp = new Date(createTimestamp(randomizer));
            NEDPosition nedPosition = createPosition(randomizer);
            CoordinateTransformation cnb = generateBodyC(randomizer);

            CoordinateTransformation nedC = cnb.inverseAndReturnNew();

            final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

            reset();
            assertTrue(mMeasurements.isEmpty());
            assertEquals(mInitializationStarted, 0);
            assertEquals(mInitializationCompleted, 0);
            assertEquals(mError, 0);
            assertEquals(mStaticIntervalDetected, 0);
            assertEquals(mDynamicIntervalDetected, 0);
            assertEquals(mGeneratedMeasurement, 0);
            assertEquals(mReset, 0);

            final MagnetometerMeasurementsGenerator generator =
                    new MagnetometerMeasurementsGenerator(this);

            // generate initial static samples
            final int initialStaticSamples = TriadStaticIntervalDetector
                    .DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics,
                    errors, hardIron, mm, wmmEstimator, random, timestamp, nedPosition,
                    cnb, noiseRandomizer);

            assertEquals(mInitializationStarted, 1);
            assertEquals(mInitializationCompleted, 1);

            final int numMeasurements = KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final int dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            for (int i = 0; i < numMeasurements; i++) {
                nedPosition = nedFrame.getPosition();
                nedC = nedFrame.getCoordinateTransformation();
                cnb = nedC.inverseAndReturnNew();

                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics,
                        errors, hardIron, mm, wmmEstimator, random, timestamp, nedPosition,
                        cnb, noiseRandomizer);

                assertEquals(mStaticIntervalDetected, i + 1);

                // generate dynamic samples
                generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics,
                        randomizer, ecefFrame, nedFrame, errors, hardIron, mm,
                        wmmEstimator, random, timestamp, nedPosition, cnb,
                        noiseRandomizer, true);

                assertEquals(mDynamicIntervalDetected, i + 1);
                assertEquals(mMeasurements.size(), i + 1);
            }

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(mReset, 1);
            assertEquals(mError, 0);

            KnownPositionAndInstantMagnetometerCalibrator calibrator =
                    new KnownPositionAndInstantMagnetometerCalibrator(
                            nedPosition, mMeasurements,
                            true);
            calibrator.setTime(timestamp);

            calibrator.calibrate();

            final Matrix estimatedHardIron = calibrator
                    .getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testProcessCalibrateAndResetWithNoiseAndGeneralCase() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            CalibrationException, NotReadyException, IOException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
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
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, MAGNETOMETER_NOISE_STD);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final Date timestamp = new Date(createTimestamp(randomizer));
            NEDPosition nedPosition = createPosition(randomizer);
            CoordinateTransformation cnb = generateBodyC(randomizer);

            CoordinateTransformation nedC = cnb.inverseAndReturnNew();

            final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

            reset();
            assertTrue(mMeasurements.isEmpty());
            assertEquals(mInitializationStarted, 0);
            assertEquals(mInitializationCompleted, 0);
            assertEquals(mError, 0);
            assertEquals(mStaticIntervalDetected, 0);
            assertEquals(mDynamicIntervalDetected, 0);
            assertEquals(mGeneratedMeasurement, 0);
            assertEquals(mReset, 0);

            final MagnetometerMeasurementsGenerator generator =
                    new MagnetometerMeasurementsGenerator(this);

            // generate initial static samples
            final int initialStaticSamples = TriadStaticIntervalDetector
                    .DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics,
                    errors, hardIron, mm, wmmEstimator, random, timestamp, nedPosition,
                    cnb, noiseRandomizer);

            assertEquals(mInitializationStarted, 1);
            assertEquals(mInitializationCompleted, 1);

            final int numMeasurements = KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final int dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            for (int i = 0; i < numMeasurements; i++) {
                nedPosition = nedFrame.getPosition();
                nedC = nedFrame.getCoordinateTransformation();
                cnb = nedC.inverseAndReturnNew();

                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics,
                        errors, hardIron, mm, wmmEstimator, random, timestamp, nedPosition,
                        cnb, noiseRandomizer);

                assertEquals(mStaticIntervalDetected, i + 1);

                // generate dynamic samples
                generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics,
                        randomizer, ecefFrame, nedFrame, errors, hardIron, mm,
                        wmmEstimator, random, timestamp, nedPosition, cnb,
                        noiseRandomizer, true);

                assertEquals(mDynamicIntervalDetected, i + 1);
                assertEquals(mMeasurements.size(), i + 1);
            }

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(mReset, 1);
            assertEquals(mError, 0);

            KnownPositionAndInstantMagnetometerCalibrator calibrator =
                    new KnownPositionAndInstantMagnetometerCalibrator(
                            nedPosition, mMeasurements,
                            false);
            calibrator.setTime(timestamp);

            calibrator.calibrate();

            final Matrix estimatedHardIron = calibrator
                    .getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testProcessSmallNoiseWithRotationAndPositionChange() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            CalibrationException, NotReadyException, IOException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
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
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, SMALL_MAGNETOMETER_NOISE_STD);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final Date timestamp = new Date(createTimestamp(randomizer));
            NEDPosition nedPosition = createPosition(randomizer);
            CoordinateTransformation cnb = generateBodyC(randomizer);

            CoordinateTransformation nedC = cnb.inverseAndReturnNew();

            final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

            reset();
            assertTrue(mMeasurements.isEmpty());
            assertEquals(mInitializationStarted, 0);
            assertEquals(mInitializationCompleted, 0);
            assertEquals(mError, 0);
            assertEquals(mStaticIntervalDetected, 0);
            assertEquals(mDynamicIntervalDetected, 0);
            assertEquals(mGeneratedMeasurement, 0);
            assertEquals(mReset, 0);

            final MagnetometerMeasurementsGenerator generator =
                    new MagnetometerMeasurementsGenerator(this);

            // generate initial static samples
            final int initialStaticSamples = TriadStaticIntervalDetector
                    .DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics,
                    errors, hardIron, mm, wmmEstimator, random, timestamp, nedPosition,
                    cnb, noiseRandomizer);

            assertEquals(mInitializationStarted, 1);
            assertEquals(mInitializationCompleted, 1);

            final int numMeasurements = KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final int dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            for (int i = 0; i < numMeasurements; i++) {
                nedPosition = nedFrame.getPosition();
                nedC = nedFrame.getCoordinateTransformation();
                cnb = nedC.inverseAndReturnNew();

                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics,
                        errors, hardIron, mm, wmmEstimator, random, timestamp, nedPosition,
                        cnb, noiseRandomizer);

                assertEquals(mStaticIntervalDetected, i + 1);

                // generate dynamic samples
                generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics,
                        randomizer, ecefFrame, nedFrame, errors, hardIron, mm,
                        wmmEstimator, random, timestamp, nedPosition, cnb,
                        noiseRandomizer, true);

                assertEquals(mDynamicIntervalDetected, i + 1);
                assertEquals(mMeasurements.size(), i + 1);
            }

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            generator.reset();

            assertEquals(mReset, 1);
            assertEquals(mError, 0);

            KnownPositionAndInstantMagnetometerCalibrator calibrator =
                    new KnownPositionAndInstantMagnetometerCalibrator(
                            nedPosition, mMeasurements,
                            true);
            calibrator.setTime(timestamp);

            calibrator.calibrate();

            final Matrix estimatedHardIron = calibrator
                    .getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testProcessErrorWithExcessiveOverallNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            IOException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
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
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, SMALL_MAGNETOMETER_NOISE_STD);

        final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix mm = generateSoftIronCommonAxis();
        assertNotNull(mm);

        final Date timestamp = new Date(createTimestamp(randomizer));
        NEDPosition nedPosition = createPosition(randomizer);
        CoordinateTransformation cnb = generateBodyC(randomizer);

        CoordinateTransformation nedC = cnb.inverseAndReturnNew();

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

        reset();
        assertTrue(mMeasurements.isEmpty());
        assertEquals(mInitializationStarted, 0);
        assertEquals(mInitializationCompleted, 0);
        assertEquals(mError, 0);
        assertEquals(mStaticIntervalDetected, 0);
        assertEquals(mDynamicIntervalDetected, 0);
        assertEquals(mGeneratedMeasurement, 0);
        assertEquals(mReset, 0);

        final MagnetometerMeasurementsGenerator generator =
                new MagnetometerMeasurementsGenerator(this);
        generator.setBaseNoiseLevelAbsoluteThreshold(Double.MIN_VALUE);

        // generate initial static samples
        final int initialStaticSamples = TriadStaticIntervalDetector
                .DEFAULT_INITIAL_STATIC_SAMPLES;
        generateStaticSamples(generator, initialStaticSamples, trueKinematics,
                errors, hardIron, mm, wmmEstimator, random, timestamp, nedPosition,
                cnb, noiseRandomizer);

        assertEquals(mInitializationStarted, 1);
        assertEquals(mInitializationCompleted, 0);
        assertEquals(mError, 1);

        final BodyMagneticFluxDensity b = generateB(hardIron.getBuffer(),
                mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition,
                cnb);
        final BodyKinematicsAndMagneticFluxDensity kb =
                new BodyKinematicsAndMagneticFluxDensity(
                        trueKinematics, b);
        assertFalse(generator.process(kb));

        generator.reset();

        assertEquals(mReset, 1);

        assertTrue(generator.process(kb));
    }

    @Test
    public void testProcessSkipStaticInterval() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            IOException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
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
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, SMALL_MAGNETOMETER_NOISE_STD);

        final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix mm = generateSoftIronCommonAxis();
        assertNotNull(mm);

        final Date timestamp = new Date(createTimestamp(randomizer));
        NEDPosition nedPosition = createPosition(randomizer);
        CoordinateTransformation cnb = generateBodyC(randomizer);

        CoordinateTransformation nedC = cnb.inverseAndReturnNew();

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

        reset();
        assertTrue(mMeasurements.isEmpty());
        assertEquals(mInitializationStarted, 0);
        assertEquals(mInitializationCompleted, 0);
        assertEquals(mError, 0);
        assertEquals(mStaticIntervalDetected, 0);
        assertEquals(mDynamicIntervalDetected, 0);
        assertEquals(mGeneratedMeasurement, 0);
        assertEquals(mReset, 0);

        final MagnetometerMeasurementsGenerator generator =
                new MagnetometerMeasurementsGenerator(this);

        // generate initial static samples
        final int initialStaticSamples = TriadStaticIntervalDetector
                .DEFAULT_INITIAL_STATIC_SAMPLES;
        generateStaticSamples(generator, initialStaticSamples, trueKinematics,
                errors, hardIron, mm, wmmEstimator, random, timestamp, nedPosition,
                cnb, noiseRandomizer);

        assertEquals(mInitializationStarted, 1);
        assertEquals(mInitializationCompleted, 1);

        final int staticPeriodLength = generator.getMinStaticSamples() / 2;
        final int dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

        nedPosition = nedFrame.getPosition();
        nedC = nedFrame.getCoordinateTransformation();
        cnb = nedC.inverseAndReturnNew();

        // generate static samples
        generateStaticSamples(generator, staticPeriodLength, trueKinematics,
                errors, hardIron, mm, wmmEstimator, random, timestamp, nedPosition,
                cnb, noiseRandomizer);

        assertEquals(mStaticIntervalDetected, 1);

        // generate dynamic samples
        generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics,
                randomizer, ecefFrame, nedFrame, errors, hardIron, mm,
                wmmEstimator, random, timestamp, nedPosition, cnb,
                noiseRandomizer, true);

        assertEquals(mDynamicIntervalDetected, 1);
        assertEquals(mStaticIntervalSkipped, 1);
    }

    @Test
    public void testProcessSkipDynamicInterval() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            IOException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
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
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, SMALL_MAGNETOMETER_NOISE_STD);

        final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix mm = generateSoftIronCommonAxis();
        assertNotNull(mm);

        final Date timestamp = new Date(createTimestamp(randomizer));
        NEDPosition nedPosition = createPosition(randomizer);
        CoordinateTransformation cnb = generateBodyC(randomizer);

        CoordinateTransformation nedC = cnb.inverseAndReturnNew();

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

        reset();
        assertTrue(mMeasurements.isEmpty());
        assertEquals(mInitializationStarted, 0);
        assertEquals(mInitializationCompleted, 0);
        assertEquals(mError, 0);
        assertEquals(mStaticIntervalDetected, 0);
        assertEquals(mDynamicIntervalDetected, 0);
        assertEquals(mGeneratedMeasurement, 0);
        assertEquals(mReset, 0);

        final MagnetometerMeasurementsGenerator generator =
                new MagnetometerMeasurementsGenerator(this);

        // generate initial static samples
        final int initialStaticSamples = TriadStaticIntervalDetector
                .DEFAULT_INITIAL_STATIC_SAMPLES;
        generateStaticSamples(generator, initialStaticSamples, trueKinematics,
                errors, hardIron, mm, wmmEstimator, random, timestamp, nedPosition,
                cnb, noiseRandomizer);

        assertEquals(mInitializationStarted, 1);
        assertEquals(mInitializationCompleted, 1);

        final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
        final int dynamicPeriodLength = 2 * generator.getMaxDynamicSamples();

        nedPosition = nedFrame.getPosition();
        nedC = nedFrame.getCoordinateTransformation();
        cnb = nedC.inverseAndReturnNew();

        // generate static samples
        generateStaticSamples(generator, staticPeriodLength, trueKinematics,
                errors, hardIron, mm, wmmEstimator, random, timestamp, nedPosition,
                cnb, noiseRandomizer);

        assertEquals(mStaticIntervalDetected, 1);

        // generate dynamic samples
        generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics,
                randomizer, ecefFrame, nedFrame, errors, hardIron, mm,
                wmmEstimator, random, timestamp, nedPosition, cnb,
                noiseRandomizer, true);

        assertEquals(mDynamicIntervalDetected, 1);
        assertEquals(mDynamicIntervalSkipped, 1);
    }

    @Override
    public void onInitializationStarted(
            final MagnetometerMeasurementsGenerator generator) {
        mInitializationStarted++;
        checkLocked(generator);

        assertEquals(generator.getStatus(),
                TriadStaticIntervalDetector.Status.INITIALIZING);
    }

    @Override
    public void onInitializationCompleted(
            final MagnetometerMeasurementsGenerator generator,
            final double baseNoiseLevel) {
        mInitializationCompleted++;
        checkLocked(generator);

        assertTrue(baseNoiseLevel > 0.0);
        assertEquals(baseNoiseLevel, generator.getBaseNoiseLevel(), 0.0);
        final Acceleration baseNoiseLevel1 = generator.getBaseNoiseLevelAsMeasurement();
        assertEquals(baseNoiseLevel1.getValue().doubleValue(), baseNoiseLevel, 0.0);
        assertEquals(baseNoiseLevel1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baseNoiseLevel2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getBaseNoiseLevelAsMeasurement(baseNoiseLevel2);
        assertEquals(baseNoiseLevel1, baseNoiseLevel2);

        assertTrue(generator.getThreshold() > 0.0);
        final Acceleration threshold1 = generator.getThresholdAsMeasurement();
        assertEquals(threshold1.getValue().doubleValue(), generator.getThreshold(),
                0.0);
        assertEquals(threshold1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration threshold2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getThresholdAsMeasurement(threshold2);
        assertEquals(threshold1, threshold2);
    }

    @Override
    public void onError(
            final MagnetometerMeasurementsGenerator generator,
            final TriadStaticIntervalDetector.ErrorReason reason) {
        mError++;
        checkLocked(generator);

        assertEquals(generator.getStatus(),
                TriadStaticIntervalDetector.Status.FAILED);
    }

    @Override
    public void onStaticIntervalDetected(
            final MagnetometerMeasurementsGenerator generator) {
        mStaticIntervalDetected++;
        checkLocked(generator);

        assertEquals(generator.getStatus(),
                TriadStaticIntervalDetector.Status.STATIC_INTERVAL);
    }

    @Override
    public void onDynamicIntervalDetected(
            final MagnetometerMeasurementsGenerator generator) {
        mDynamicIntervalDetected++;
        checkLocked(generator);

        assertEquals(generator.getStatus(),
                TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL);
    }

    @Override
    public void onStaticIntervalSkipped(
            final MagnetometerMeasurementsGenerator generator) {
        mStaticIntervalSkipped++;
        checkLocked(generator);

        assertEquals(generator.getStatus(),
                TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL);
    }

    @Override
    public void onDynamicIntervalSkipped(
            final MagnetometerMeasurementsGenerator generator) {
        mDynamicIntervalSkipped++;
        checkLocked(generator);

        assertEquals(generator.getStatus(),
                TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL);
    }

    @Override
    public void onGeneratedMeasurement(
            final MagnetometerMeasurementsGenerator generator,
            final StandardDeviationBodyMagneticFluxDensity measurement) {
        mGeneratedMeasurement++;
        mMeasurements.add(measurement);
        checkLocked(generator);
    }

    @Override
    public void onReset(
            final MagnetometerMeasurementsGenerator generator) {
        mReset++;

        assertEquals(generator.getStatus(),
                TriadStaticIntervalDetector.Status.IDLE);
    }

    private void reset() {
        mMeasurements.clear();

        mInitializationStarted = 0;
        mInitializationCompleted = 0;
        mError = 0;
        mStaticIntervalDetected = 0;
        mDynamicIntervalDetected = 0;
        mStaticIntervalSkipped = 0;
        mDynamicIntervalSkipped = 0;
        mGeneratedMeasurement = 0;
        mReset = 0;
    }

    private void checkLocked(final MagnetometerMeasurementsGenerator generator) {
        assertTrue(generator.isRunning());
        try {
            generator.setMinStaticSamples(0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            generator.setMaxDynamicSamples(0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            generator.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            generator.setWindowSize(0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            generator.setInitialStaticSamples(0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            generator.setThresholdFactor(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            generator.setInstantaneousNoiseLevelFactor(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            generator.setBaseNoiseLevelAbsoluteThreshold(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            generator.setBaseNoiseLevelAbsoluteThreshold(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            generator.process(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            generator.reset();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
    }

    private static BodyMagneticFluxDensity generateB(
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

        return measuredMagnetic;
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

    private static Matrix generateSoftIronCommonAxis() {
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        for (int col = 0; col < mm.getColumns(); col++) {
            for (int row = 0; row < mm.getRows(); row++) {
                if (row > col) {
                    mm.setElementAt(row, col, 0.0);
                }
            }
        }
        return mm;
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

    private Matrix generateMaCommonAxis() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                0.0, -600e-6, 250e-6,
                0.0, 0.0, 450e-6
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

    private void generateStaticSamples(
            final MagnetometerMeasurementsGenerator generator,
            final int numSamples,
            final BodyKinematics trueKinematics,
            final IMUErrors errors,
            final Matrix hardIron,
            final Matrix mm,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final Random random,
            final Date timestamp,
            final NEDPosition nedPosition,
            final CoordinateTransformation cnb,
            final GaussianRandomizer noiseRandomizer) throws LockedException {
        final BodyKinematics measuredKinematics = new BodyKinematics();
        for (int i = 0; i < numSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);

            final BodyMagneticFluxDensity b = generateB(hardIron.getBuffer(),
                    mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition,
                    cnb);
            final BodyKinematicsAndMagneticFluxDensity kb =
                    new BodyKinematicsAndMagneticFluxDensity(
                            measuredKinematics, b);
            assertTrue(generator.process(kb));
        }
    }

    private void generateDynamicSamples(
            final MagnetometerMeasurementsGenerator generator,
            final int numSamples,
            final BodyKinematics trueKinematics,
            final UniformRandomizer randomizer,
            final ECEFFrame ecefFrame,
            final NEDFrame nedFrame,
            final IMUErrors errors,
            final Matrix hardIron,
            final Matrix mm,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final Random random,
            final Date timestamp,
            final NEDPosition nedPosition,
            final CoordinateTransformation cnb,
            final GaussianRandomizer noiseRandomizer,
            final boolean changePosition)
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {

        final double deltaX = changePosition ?
                randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;
        final double deltaY = changePosition ?
                randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;
        final double deltaZ = changePosition ?
                randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;

        final double deltaRoll = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaPitch = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaYaw = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final double ecefX = ecefFrame.getX();
        final double ecefY = ecefFrame.getY();
        final double ecefZ = ecefFrame.getZ();

        final CoordinateTransformation nedC = nedFrame.getCoordinateTransformation();

        final double roll = nedC.getRollEulerAngle();
        final double pitch = nedC.getPitchEulerAngle();
        final double yaw = nedC.getYawEulerAngle();

        NEDFrame oldNedFrame = new NEDFrame(nedFrame);
        NEDFrame newNedFrame = new NEDFrame();
        ECEFFrame oldEcefFrame = new ECEFFrame(ecefFrame);
        ECEFFrame newEcefFrame = new ECEFFrame();

        double oldEcefX = ecefX - deltaX;
        double oldEcefY = ecefY - deltaY;
        double oldEcefZ = ecefZ - deltaZ;
        double oldRoll = roll - deltaRoll;
        double oldPitch = pitch - deltaPitch;
        double oldYaw = yaw - deltaYaw;

        final BodyKinematics measuredKinematics = new BodyKinematics();

        for (int i = 0; i < numSamples; i++) {
            final double newRoll = oldRoll + deltaRoll;
            final double newPitch = oldPitch + deltaPitch;
            final double newYaw = oldYaw + deltaYaw;
            final CoordinateTransformation newNedC =
                    new CoordinateTransformation(
                            newRoll, newPitch, newYaw,
                            FrameType.BODY_FRAME,
                            FrameType.LOCAL_NAVIGATION_FRAME);
            final NEDPosition newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final double newEcefX = oldEcefX + deltaX;
            final double newEcefY = oldEcefY + deltaY;
            final double newEcefZ = oldEcefZ + deltaZ;

            newEcefFrame.setCoordinates(newEcefX, newEcefY, newEcefZ);

            ECEFtoNEDFrameConverter.convertECEFtoNED(newEcefFrame, newNedFrame);

            // update true kinematics using new position and rotation
            ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS,
                    newEcefFrame, oldEcefFrame, trueKinematics);

            // add error to true kinematics
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);

            final BodyMagneticFluxDensity b = generateB(hardIron.getBuffer(),
                    mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition,
                    cnb);
            final BodyKinematicsAndMagneticFluxDensity kb =
                    new BodyKinematicsAndMagneticFluxDensity(
                            measuredKinematics, b);
            assertTrue(generator.process(kb));

            oldNedFrame.copyFrom(newNedFrame);
            oldEcefFrame.copyFrom(newEcefFrame);
            oldRoll = newRoll;
            oldPitch = newPitch;
            oldYaw = newYaw;
            oldEcefX = newEcefX;
            oldEcefY = newEcefY;
            oldEcefZ = newEcefZ;
        }

        // update current ECEF and NED frames
        ecefFrame.copyFrom(newEcefFrame);
        nedFrame.copyFrom(newNedFrame);

        // after dynamic sequence finishes, update true kinematics for a
        // static sequence at current frame
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS,
                newEcefFrame, newEcefFrame, trueKinematics);
    }

}
