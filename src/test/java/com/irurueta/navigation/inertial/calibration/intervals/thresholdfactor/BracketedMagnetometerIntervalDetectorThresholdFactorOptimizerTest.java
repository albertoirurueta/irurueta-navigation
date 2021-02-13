/*
 * Copyright (C) 2021 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NavigationException;
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
import com.irurueta.navigation.inertial.calibration.generators.MagnetometerMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.generators.MagnetometerMeasurementsGeneratorListener;
import com.irurueta.navigation.inertial.calibration.generators.MeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.magnetometer.KnownFrameMagnetometerNonLinearLeastSquaresCalibrator;
import com.irurueta.navigation.inertial.calibration.magnetometer.KnownPositionAndInstantMagnetometerCalibrator;
import com.irurueta.navigation.inertial.calibration.magnetometer.PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator;
import com.irurueta.navigation.inertial.calibration.noise.WindowedTriadNoiseEstimator;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.numerical.optimization.BrentSingleOptimizer;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
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
import static org.mockito.Mockito.mock;

public class BracketedMagnetometerIntervalDetectorThresholdFactorOptimizerTest implements
        IntervalDetectorThresholdFactorOptimizerListener<BodyKinematicsAndMagneticFluxDensity,
                MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource> {

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

    private final List<BodyKinematicsAndMagneticFluxDensity> mBodyKinematicsAndMagneticFluxDensities =
            new ArrayList<>();

    private final MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource mDataSource =
            new MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource() {
                @Override
                public int count() {
                    return mBodyKinematicsAndMagneticFluxDensities.size();
                }

                @Override
                public BodyKinematicsAndMagneticFluxDensity getAt(final int index) {
                    return mBodyKinematicsAndMagneticFluxDensities.get(index);
                }
            };

    private final MagnetometerMeasurementsGeneratorListener mGeneratorListener =
            new MagnetometerMeasurementsGeneratorListener() {
                @Override
                public void onInitializationStarted(
                        final MagnetometerMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onInitializationCompleted(
                        final MagnetometerMeasurementsGenerator generator,
                        final double baseNoiseLevel) {
                    // not used
                }

                @Override
                public void onError(
                        final MagnetometerMeasurementsGenerator generator,
                        final TriadStaticIntervalDetector.ErrorReason reason) {
                    // not used
                }

                @Override
                public void onStaticIntervalDetected(
                        final MagnetometerMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onDynamicIntervalDetected(
                        final MagnetometerMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onStaticIntervalSkipped(
                        final MagnetometerMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onDynamicIntervalSkipped(
                        final MagnetometerMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onGeneratedMeasurement(
                        final MagnetometerMeasurementsGenerator generator,
                        final StandardDeviationBodyMagneticFluxDensity measurement) {
                    mGeneratorMeasurements.add(measurement);
                }

                @Override
                public void onReset(final MagnetometerMeasurementsGenerator generator) {
                    // not used
                }
            };

    private final List<StandardDeviationBodyMagneticFluxDensity> mGeneratorMeasurements =
            new ArrayList<>();

    private int mStart;

    private int mEnd;

    @Test
    public void testConstructor1() {
        final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default values
        assertNotNull(optimizer.getMseOptimizer());
        assertNull(optimizer.getCalibrator());
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);
        assertFalse(optimizer.isReady());
        assertNull(optimizer.getDataSource());
        assertFalse(optimizer.isRunning());
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                optimizer.getTimeInterval(), 0.0);
        final Time timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(timeInterval1.getUnit(), TimeUnit.SECOND);
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        optimizer.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
                optimizer.getMinStaticSamples());
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
                optimizer.getMaxDynamicSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE,
                optimizer.getWindowSize());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
                optimizer.getInitialStaticSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                optimizer.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        final Acceleration acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration1.getUnit());
        final Acceleration acceleration2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevel(),
                0.0);
        final Acceleration acceleration3 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, acceleration3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration3.getUnit());
        final Acceleration acceleration4 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration4);
        assertEquals(acceleration3, acceleration4);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelPsd(),
                0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelRootPsd(),
                0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);
        final Acceleration acceleration5 = optimizer.getThresholdAsMeasurement();
        assertEquals(0.0, acceleration5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration5.getUnit());
        final Acceleration acceleration6 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getThresholdAsMeasurement(acceleration6);
        assertEquals(acceleration5, acceleration6);
        assertNull(optimizer.getEstimatedHardIron());
        assertNull(optimizer.getEstimatedMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
    }

    @Test
    public void testConstructor2() {
        final MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
                mock(MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource.class);

        final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(
                        dataSource);

        // check default values
        assertNotNull(optimizer.getMseOptimizer());
        assertNull(optimizer.getCalibrator());
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);
        assertFalse(optimizer.isReady());
        assertSame(dataSource, optimizer.getDataSource());
        assertFalse(optimizer.isRunning());
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                optimizer.getTimeInterval(), 0.0);
        final Time timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(timeInterval1.getUnit(), TimeUnit.SECOND);
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        optimizer.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
                optimizer.getMinStaticSamples());
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
                optimizer.getMaxDynamicSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE,
                optimizer.getWindowSize());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
                optimizer.getInitialStaticSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                optimizer.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        final Acceleration acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration1.getUnit());
        final Acceleration acceleration2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevel(),
                0.0);
        final Acceleration acceleration3 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, acceleration3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration3.getUnit());
        final Acceleration acceleration4 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration4);
        assertEquals(acceleration3, acceleration4);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelPsd(),
                0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelRootPsd(),
                0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);
        final Acceleration acceleration5 = optimizer.getThresholdAsMeasurement();
        assertEquals(0.0, acceleration5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration5.getUnit());
        final Acceleration acceleration6 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getThresholdAsMeasurement(acceleration6);
        assertEquals(acceleration5, acceleration6);
        assertNull(optimizer.getEstimatedHardIron());
        assertNull(optimizer.getEstimatedMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
    }

    @Test
    public void testConstructor3() {
        final KnownPositionAndInstantMagnetometerCalibrator calibrator =
                new KnownPositionAndInstantMagnetometerCalibrator();

        BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(
                        calibrator);

        // check default values
        assertNotNull(optimizer.getMseOptimizer());
        assertSame(calibrator, optimizer.getCalibrator());
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);
        assertFalse(optimizer.isReady());
        assertNull(optimizer.getDataSource());
        assertFalse(optimizer.isRunning());
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                optimizer.getTimeInterval(), 0.0);
        final Time timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(timeInterval1.getUnit(), TimeUnit.SECOND);
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        optimizer.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
                optimizer.getMinStaticSamples());
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
                optimizer.getMaxDynamicSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE,
                optimizer.getWindowSize());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
                optimizer.getInitialStaticSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                optimizer.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        final Acceleration acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration1.getUnit());
        final Acceleration acceleration2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevel(),
                0.0);
        final Acceleration acceleration3 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, acceleration3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration3.getUnit());
        final Acceleration acceleration4 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration4);
        assertEquals(acceleration3, acceleration4);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelPsd(),
                0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelRootPsd(),
                0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);
        final Acceleration acceleration5 = optimizer.getThresholdAsMeasurement();
        assertEquals(0.0, acceleration5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration5.getUnit());
        final Acceleration acceleration6 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getThresholdAsMeasurement(acceleration6);
        assertEquals(acceleration5, acceleration6);
        assertNull(optimizer.getEstimatedHardIron());
        assertNull(optimizer.getEstimatedMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());

        // Force IllegalArgumentException
        optimizer = null;
        try {
            optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(
                    new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(optimizer);
    }

    @Test
    public void testConstructor4() {
        final MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
                mock(MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource.class);

        final KnownPositionAndInstantMagnetometerCalibrator calibrator =
                new KnownPositionAndInstantMagnetometerCalibrator();

        BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(
                        dataSource, calibrator);

        // check default values
        assertNotNull(optimizer.getMseOptimizer());
        assertSame(calibrator, optimizer.getCalibrator());
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);
        assertTrue(optimizer.isReady());
        assertSame(dataSource, optimizer.getDataSource());
        assertFalse(optimizer.isRunning());
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                optimizer.getTimeInterval(), 0.0);
        final Time timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(timeInterval1.getUnit(), TimeUnit.SECOND);
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        optimizer.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
                optimizer.getMinStaticSamples());
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
                optimizer.getMaxDynamicSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE,
                optimizer.getWindowSize());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
                optimizer.getInitialStaticSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                optimizer.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        final Acceleration acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration1.getUnit());
        final Acceleration acceleration2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevel(),
                0.0);
        final Acceleration acceleration3 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, acceleration3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration3.getUnit());
        final Acceleration acceleration4 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration4);
        assertEquals(acceleration3, acceleration4);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelPsd(),
                0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelRootPsd(),
                0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);
        final Acceleration acceleration5 = optimizer.getThresholdAsMeasurement();
        assertEquals(0.0, acceleration5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration5.getUnit());
        final Acceleration acceleration6 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getThresholdAsMeasurement(acceleration6);
        assertEquals(acceleration5, acceleration6);
        assertNull(optimizer.getEstimatedHardIron());
        assertNull(optimizer.getEstimatedMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());

        // Force IllegalArgumentException
        optimizer = null;
        try {
            optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(
                    dataSource, new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(optimizer);
    }

    @Test
    public void testConstructor5() {
        final BrentSingleOptimizer mseOptimizer = new BrentSingleOptimizer();

        final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(
                        mseOptimizer);

        // check default values
        assertSame(mseOptimizer, optimizer.getMseOptimizer());
        assertNull(optimizer.getCalibrator());
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);
        assertFalse(optimizer.isReady());
        assertNull(optimizer.getDataSource());
        assertFalse(optimizer.isRunning());
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                optimizer.getTimeInterval(), 0.0);
        final Time timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(timeInterval1.getUnit(), TimeUnit.SECOND);
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        optimizer.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
                optimizer.getMinStaticSamples());
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
                optimizer.getMaxDynamicSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE,
                optimizer.getWindowSize());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
                optimizer.getInitialStaticSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                optimizer.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        final Acceleration acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration1.getUnit());
        final Acceleration acceleration2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevel(),
                0.0);
        final Acceleration acceleration3 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, acceleration3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration3.getUnit());
        final Acceleration acceleration4 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration4);
        assertEquals(acceleration3, acceleration4);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelPsd(),
                0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelRootPsd(),
                0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);
        final Acceleration acceleration5 = optimizer.getThresholdAsMeasurement();
        assertEquals(0.0, acceleration5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration5.getUnit());
        final Acceleration acceleration6 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getThresholdAsMeasurement(acceleration6);
        assertEquals(acceleration5, acceleration6);
        assertNull(optimizer.getEstimatedHardIron());
        assertNull(optimizer.getEstimatedMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
    }

    @Test
    public void testConstructor6() {
        final MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
                mock(MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource.class);
        final BrentSingleOptimizer mseOptimizer = new BrentSingleOptimizer();

        final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(
                        dataSource, mseOptimizer);

        // check default values
        assertSame(mseOptimizer, optimizer.getMseOptimizer());
        assertNull(optimizer.getCalibrator());
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);
        assertFalse(optimizer.isReady());
        assertSame(dataSource, optimizer.getDataSource());
        assertFalse(optimizer.isRunning());
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                optimizer.getTimeInterval(), 0.0);
        final Time timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(timeInterval1.getUnit(), TimeUnit.SECOND);
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        optimizer.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
                optimizer.getMinStaticSamples());
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
                optimizer.getMaxDynamicSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE,
                optimizer.getWindowSize());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
                optimizer.getInitialStaticSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                optimizer.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        final Acceleration acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration1.getUnit());
        final Acceleration acceleration2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevel(),
                0.0);
        final Acceleration acceleration3 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, acceleration3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration3.getUnit());
        final Acceleration acceleration4 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration4);
        assertEquals(acceleration3, acceleration4);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelPsd(),
                0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelRootPsd(),
                0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);
        final Acceleration acceleration5 = optimizer.getThresholdAsMeasurement();
        assertEquals(0.0, acceleration5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration5.getUnit());
        final Acceleration acceleration6 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getThresholdAsMeasurement(acceleration6);
        assertEquals(acceleration5, acceleration6);
        assertNull(optimizer.getEstimatedHardIron());
        assertNull(optimizer.getEstimatedMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
    }

    @Test
    public void testConstructor7() {
        final KnownPositionAndInstantMagnetometerCalibrator calibrator =
                new KnownPositionAndInstantMagnetometerCalibrator();
        final BrentSingleOptimizer mseOptimizer = new BrentSingleOptimizer();

        BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(
                        calibrator, mseOptimizer);

        // check default values
        assertSame(mseOptimizer, optimizer.getMseOptimizer());
        assertSame(calibrator, optimizer.getCalibrator());
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);
        assertFalse(optimizer.isReady());
        assertNull(optimizer.getDataSource());
        assertFalse(optimizer.isRunning());
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                optimizer.getTimeInterval(), 0.0);
        final Time timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(timeInterval1.getUnit(), TimeUnit.SECOND);
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        optimizer.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
                optimizer.getMinStaticSamples());
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
                optimizer.getMaxDynamicSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE,
                optimizer.getWindowSize());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
                optimizer.getInitialStaticSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                optimizer.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        final Acceleration acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration1.getUnit());
        final Acceleration acceleration2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevel(),
                0.0);
        final Acceleration acceleration3 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, acceleration3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration3.getUnit());
        final Acceleration acceleration4 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration4);
        assertEquals(acceleration3, acceleration4);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelPsd(),
                0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelRootPsd(),
                0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);
        final Acceleration acceleration5 = optimizer.getThresholdAsMeasurement();
        assertEquals(0.0, acceleration5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration5.getUnit());
        final Acceleration acceleration6 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getThresholdAsMeasurement(acceleration6);
        assertEquals(acceleration5, acceleration6);
        assertNull(optimizer.getEstimatedHardIron());
        assertNull(optimizer.getEstimatedMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());

        // Force IllegalArgumentException
        optimizer = null;
        try {
            optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(
                    new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(),
                    mseOptimizer);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(optimizer);
    }

    @Test
    public void testConstructor8() {
        final MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
                mock(MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource.class);

        final KnownPositionAndInstantMagnetometerCalibrator calibrator =
                new KnownPositionAndInstantMagnetometerCalibrator();
        final BrentSingleOptimizer mseOptimizer = new BrentSingleOptimizer();

        BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(
                        dataSource, calibrator, mseOptimizer);

        // check default values
        assertSame(mseOptimizer, optimizer.getMseOptimizer());
        assertSame(calibrator, optimizer.getCalibrator());
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);
        assertTrue(optimizer.isReady());
        assertSame(dataSource, optimizer.getDataSource());
        assertFalse(optimizer.isRunning());
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                optimizer.getTimeInterval(), 0.0);
        final Time timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(timeInterval1.getUnit(), TimeUnit.SECOND);
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        optimizer.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
                optimizer.getMinStaticSamples());
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
                optimizer.getMaxDynamicSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE,
                optimizer.getWindowSize());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
                optimizer.getInitialStaticSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                optimizer.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        final Acceleration acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration1.getUnit());
        final Acceleration acceleration2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevel(),
                0.0);
        final Acceleration acceleration3 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, acceleration3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration3.getUnit());
        final Acceleration acceleration4 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration4);
        assertEquals(acceleration3, acceleration4);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelPsd(),
                0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelRootPsd(),
                0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);
        final Acceleration acceleration5 = optimizer.getThresholdAsMeasurement();
        assertEquals(0.0, acceleration5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration5.getUnit());
        final Acceleration acceleration6 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getThresholdAsMeasurement(acceleration6);
        assertEquals(acceleration5, acceleration6);
        assertNull(optimizer.getEstimatedHardIron());
        assertNull(optimizer.getEstimatedMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());

        // Force IllegalArgumentException
        optimizer = null;
        try {
            optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(
                    dataSource, new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(),
                    mseOptimizer);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(optimizer);
    }

    @Test
    public void testGetSetMseOptimizer() throws LockedException {
        final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNotNull(optimizer.getMseOptimizer());

        // set new value
        final BrentSingleOptimizer mseOptimizer = new BrentSingleOptimizer();
        optimizer.setMseOptimizer(mseOptimizer);

        // check
        assertSame(mseOptimizer, optimizer.getMseOptimizer());

        // set null value
        optimizer.setMseOptimizer(null);

        // check
        assertNull(optimizer.getMseOptimizer());
    }

    @Test
    public void testIsReady() throws LockedException {
        final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertFalse(optimizer.isReady());

        // set data source
        final MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
                mock(MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource.class);
        optimizer.setDataSource(dataSource);

        // check
        assertFalse(optimizer.isReady());

        // set calibrator
        final KnownPositionAndInstantMagnetometerCalibrator calibrator =
                new KnownPositionAndInstantMagnetometerCalibrator();
        optimizer.setCalibrator(calibrator);

        // check
        assertTrue(optimizer.isReady());

        // unset quality score mapper
        optimizer.setQualityScoreMapper(null);

        // check
        assertFalse(optimizer.isReady());

        // set quality score mapper
        optimizer.setQualityScoreMapper(new DefaultMagnetometerQualityScoreMapper());

        // check
        assertTrue(optimizer.isReady());

        // unset MSE optimizer
        optimizer.setMseOptimizer(null);

        // check
        assertFalse(optimizer.isReady());
    }

    @Test
    public void testGetSetCalibrator() throws LockedException {
        final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNull(optimizer.getCalibrator());

        // set new value
        final KnownPositionAndInstantMagnetometerCalibrator calibrator =
                new KnownPositionAndInstantMagnetometerCalibrator();

        optimizer.setCalibrator(calibrator);

        // check
        assertSame(calibrator, optimizer.getCalibrator());

        // Force IllegalArgumentException
        try {
            optimizer.setCalibrator(
                    new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetQualityScoreMapper() throws LockedException {
        final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());

        // set new value
        //noinspection unchecked
        final QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity> qualityScoreMapper =
                mock(QualityScoreMapper.class);
        optimizer.setQualityScoreMapper(qualityScoreMapper);

        // check
        assertSame(qualityScoreMapper, optimizer.getQualityScoreMapper());
    }

    @Test
    public void testGetSetThresholdFactorRange() throws LockedException {
        final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default values
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);

        // set new values
        optimizer.setThresholdFactorRange(0.0, 1.0);

        // check
        assertEquals(0.0, optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(1.0, optimizer.getMaxThresholdFactor(), 0.0);

        // Force IllegalArgumentException
        try {
            optimizer.setThresholdFactorRange(-1.0, 1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            optimizer.setThresholdFactorRange(1.0, -1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            optimizer.setThresholdFactorRange(2.0, 1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetDataSource() throws LockedException {
        final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default values
        assertNull(optimizer.getDataSource());

        // set new value
        final MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
                mock(MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource.class);

        optimizer.setDataSource(dataSource);

        // check
        assertSame(dataSource, optimizer.getDataSource());
    }

    @Test
    public void testGetSetTimeInterval() throws LockedException {
        final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                optimizer.getTimeInterval(), 0.0);

        // set new value
        final double timeInterval = 0.01;
        optimizer.setTimeInterval(timeInterval);

        // check
        assertEquals(timeInterval, optimizer.getTimeInterval(), 0.0);

        // Force IllegalArgumentException
        try {
            optimizer.setTimeInterval(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetTimeIntervalAsTime() throws LockedException {
        final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        final Time timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(timeInterval1.getUnit(), TimeUnit.SECOND);

        // set new value
        final Time timeInterval2 = new Time(0.01, TimeUnit.SECOND);
        optimizer.setTimeInterval(timeInterval2);

        // check
        final Time timeInterval3 = optimizer.getTimeIntervalAsTime();
        final Time timeInterval4 = new Time(1.0, TimeUnit.DAY);
        optimizer.getTimeIntervalAsTime(timeInterval4);

        assertEquals(timeInterval2, timeInterval3);
        assertEquals(timeInterval2, timeInterval4);

        // Force IllegalArgumentException
        try {
            optimizer.setTimeInterval(new Time(-1.0, TimeUnit.SECOND));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMinStaticSamples() throws LockedException {
        final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES,
                optimizer.getMinStaticSamples());

        // set new value
        final int minStaticSamples = 50;
        optimizer.setMinStaticSamples(minStaticSamples);

        // check
        assertEquals(minStaticSamples, optimizer.getMinStaticSamples());

        // Force IllegalArgumentException
        try {
            optimizer.setMinStaticSamples(1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMaxDynamicSamples() throws LockedException {
        final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES,
                optimizer.getMaxDynamicSamples());

        // set new value
        final int maxDynamicSamples = 500;
        optimizer.setMaxDynamicSamples(maxDynamicSamples);

        // check
        assertEquals(maxDynamicSamples, optimizer.getMaxDynamicSamples());

        // Force IllegalArgumentException
        try {
            optimizer.setMaxDynamicSamples(1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetWindowSize() throws LockedException {
        final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE,
                optimizer.getWindowSize());

        // set new value
        final int windowSize = 51;
        optimizer.setWindowSize(windowSize);

        // check
        assertEquals(windowSize, optimizer.getWindowSize());

        // Force IllegalArgumentException
        try {
            optimizer.setWindowSize(1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetInitialStaticSamples() throws LockedException {
        final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES,
                optimizer.getInitialStaticSamples());

        // set new value
        final int initialStaticSamples = 100;
        optimizer.setInitialStaticSamples(initialStaticSamples);

        // check
        assertEquals(initialStaticSamples, optimizer.getInitialStaticSamples());

        // Force IllegalArgumentException
        try {
            optimizer.setInitialStaticSamples(1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetInstantaneousNoiseLevelFactor() throws LockedException {
        final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                optimizer.getInstantaneousNoiseLevelFactor(), 0.0);

        // set new value
        final double instantaneousNoiseLevelFactor = 3.0;
        optimizer.setInstantaneousNoiseLevelFactor(instantaneousNoiseLevelFactor);

        // check
        assertEquals(instantaneousNoiseLevelFactor,
                optimizer.getInstantaneousNoiseLevelFactor(), 0.0);

        // Force IllegalArgumentException
        try {
            optimizer.setInstantaneousNoiseLevelFactor(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetBaseNoiseLevelAbsoluteThreshold()
            throws LockedException {
        final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);

        // set new value
        final double baseNoiseLevelAbsoluteThreshold = 1e-5;
        optimizer.setBaseNoiseLevelAbsoluteThreshold(baseNoiseLevelAbsoluteThreshold);

        // check
        assertEquals(baseNoiseLevelAbsoluteThreshold,
                optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);

        // Force IllegalArgumentException
        try {
            optimizer.setBaseNoiseLevelAbsoluteThreshold(
                    0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetBaseNoiseLevelAbsoluteThresholdAsMeasurement()
            throws LockedException {
        final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        final Acceleration acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                acceleration1.getUnit());

        // set new value
        final double baseNoiseLevelAbsoluteThreshold = 1e-5;
        final Acceleration acceleration2 = new Acceleration(
                baseNoiseLevelAbsoluteThreshold,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        optimizer.setBaseNoiseLevelAbsoluteThreshold(acceleration2);

        // check
        final Acceleration acceleration3 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        final Acceleration acceleration4 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration4);

        assertEquals(acceleration2, acceleration3);
        assertEquals(acceleration2, acceleration4);
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNull(optimizer.getListener());

        // set new value
        optimizer.setListener(this);

        // check
        assertSame(this, optimizer.getListener());
    }

    @Test
    public void testOptimizeGeneralWithNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            IntervalDetectorThresholdFactorOptimizerException, CalibrationException, IOException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mBodyKinematicsAndMagneticFluxDensities.clear();
            mGeneratorMeasurements.clear();

            // generate measurements

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final int numMeasurements = KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronGeneral();
            assertNotNull(mm);
            final Date timestamp = new Date(createTimestamp(randomizer));
            NEDPosition nedPosition = createPosition(randomizer);
            assertTrue(generateBodyKinematicsAndMagneticFluxDensity(
                    false, hardIron, mm, accelNoiseRootPSD,
                    gyroNoiseRootPSD, numMeasurements, timestamp, nedPosition,
                    MAGNETOMETER_NOISE_STD));

            // configure calibrator and data source
            KnownPositionAndInstantMagnetometerCalibrator calibrator =
                    new KnownPositionAndInstantMagnetometerCalibrator();
            calibrator.setPosition(nedPosition);
            calibrator.setCommonAxisUsed(false);
            calibrator.setTime(timestamp);

            // create optimizer
            final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                    new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(
                            mDataSource, calibrator);
            optimizer.setListener(this);

            reset();
            assertEquals(0, mStart);
            assertEquals(0, mEnd);

            final double thresholdFactor = optimizer.optimize();

            // check optimization results
            assertEquals(1, mStart);
            assertEquals(1, mEnd);
            assertEquals(thresholdFactor, optimizer.getOptimalThresholdFactor(),
                    0.0);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevel() > 0.0);
            final Acceleration accelerometerBaseNoiseLevel1 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
            assertEquals(accelerometerBaseNoiseLevel1.getValue().doubleValue(),
                    optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                    accelerometerBaseNoiseLevel1.getUnit());
            final Acceleration accelerometerBaseNoiseLevel2 = new Acceleration(
                    1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(
                    accelerometerBaseNoiseLevel2);
            assertEquals(accelerometerBaseNoiseLevel1, accelerometerBaseNoiseLevel2);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getAccelerometerBaseNoiseLevelPsd()),
                    optimizer.getAccelerometerBaseNoiseLevelRootPsd(),
                    ABSOLUTE_ERROR);
            assertTrue(optimizer.getThreshold() > 0.0);
            final Acceleration threshold1 = optimizer.getThresholdAsMeasurement();
            assertEquals(optimizer.getThreshold(), threshold1.getValue().doubleValue(),
                    0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
            final Acceleration threshold2 = new Acceleration(
                    1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getThresholdAsMeasurement(threshold2);
            assertEquals(threshold1, threshold2);
            assertNotNull(optimizer.getEstimatedHardIron());

            final Matrix optimalHardIron = Matrix.newFromArray(optimizer.getEstimatedHardIron());
            final Matrix optimalMm = optimizer.getEstimatedMm();

            assertNotNull(optimalHardIron);
            assertNotNull(optimalMm);

            if (!hardIron.equals(optimalHardIron, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(optimalMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(optimalHardIron, LARGE_ABSOLUTE_ERROR));
            assertTrue(mm.equals(optimalMm, VERY_LARGE_ABSOLUTE_ERROR));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            final MagnetometerMeasurementsGenerator generator =
                    new MagnetometerMeasurementsGenerator(mGeneratorListener);

            generator.setThresholdFactor(thresholdFactor);

            for (BodyKinematicsAndMagneticFluxDensity bodyKinematics : mBodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(bodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            calibrator.setMeasurements(mGeneratorMeasurements);

            // calibrate
            calibrator.calibrate();

            // check calibration result
            final Matrix estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
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
    public void testOptimizeCommonAxisSmallNoiseOnlyRotation() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            IntervalDetectorThresholdFactorOptimizerException, CalibrationException, IOException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mBodyKinematicsAndMagneticFluxDensities.clear();
            mGeneratorMeasurements.clear();

            // generate measurements

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final int numMeasurements = KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);
            final Date timestamp = new Date(createTimestamp(randomizer));
            NEDPosition nedPosition = createPosition(randomizer);
            assertTrue(generateBodyKinematicsAndMagneticFluxDensity(
                    false, hardIron, mm, accelNoiseRootPSD,
                    gyroNoiseRootPSD, numMeasurements, timestamp, nedPosition,
                    SMALL_MAGNETOMETER_NOISE_STD));

            // configure calibrator and data source
            KnownPositionAndInstantMagnetometerCalibrator calibrator =
                    new KnownPositionAndInstantMagnetometerCalibrator();
            calibrator.setPosition(nedPosition);
            calibrator.setCommonAxisUsed(true);
            calibrator.setTime(timestamp);

            // create optimizer
            final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                    new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(
                            mDataSource, calibrator);
            optimizer.setListener(this);

            reset();
            assertEquals(0, mStart);
            assertEquals(0, mEnd);

            final double thresholdFactor = optimizer.optimize();

            // check optimization results
            assertEquals(1, mStart);
            assertEquals(1, mEnd);
            assertEquals(thresholdFactor, optimizer.getOptimalThresholdFactor(),
                    0.0);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevel() > 0.0);
            final Acceleration accelerometerBaseNoiseLevel1 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
            assertEquals(accelerometerBaseNoiseLevel1.getValue().doubleValue(),
                    optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                    accelerometerBaseNoiseLevel1.getUnit());
            final Acceleration accelerometerBaseNoiseLevel2 = new Acceleration(
                    1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(
                    accelerometerBaseNoiseLevel2);
            assertEquals(accelerometerBaseNoiseLevel1, accelerometerBaseNoiseLevel2);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getAccelerometerBaseNoiseLevelPsd()),
                    optimizer.getAccelerometerBaseNoiseLevelRootPsd(),
                    ABSOLUTE_ERROR);
            assertTrue(optimizer.getThreshold() > 0.0);
            final Acceleration threshold1 = optimizer.getThresholdAsMeasurement();
            assertEquals(optimizer.getThreshold(), threshold1.getValue().doubleValue(),
                    0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
            final Acceleration threshold2 = new Acceleration(
                    1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getThresholdAsMeasurement(threshold2);
            assertEquals(threshold1, threshold2);
            assertNotNull(optimizer.getEstimatedHardIron());

            final Matrix optimalHardIron = Matrix.newFromArray(optimizer.getEstimatedHardIron());
            final Matrix optimalMm = optimizer.getEstimatedMm();

            assertNotNull(optimalHardIron);
            assertNotNull(optimalMm);

            if (!hardIron.equals(optimalHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(optimalMm, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(optimalHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(optimalMm, LARGE_ABSOLUTE_ERROR));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            final MagnetometerMeasurementsGenerator generator =
                    new MagnetometerMeasurementsGenerator(mGeneratorListener);

            generator.setThresholdFactor(thresholdFactor);

            for (BodyKinematicsAndMagneticFluxDensity bodyKinematics : mBodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(bodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            calibrator.setMeasurements(mGeneratorMeasurements);

            // calibrate
            calibrator.calibrate();

            // check calibration result
            final Matrix estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
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
    public void testOptimizeCommonAxisSmallNoiseWithRotationAndPositionChange() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            IntervalDetectorThresholdFactorOptimizerException, CalibrationException, IOException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mBodyKinematicsAndMagneticFluxDensities.clear();
            mGeneratorMeasurements.clear();

            // generate measurements

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final int numMeasurements = KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);
            final Date timestamp = new Date(createTimestamp(randomizer));
            NEDPosition nedPosition = createPosition(randomizer);
            assertTrue(generateBodyKinematicsAndMagneticFluxDensity(
                    true, hardIron, mm, accelNoiseRootPSD,
                    gyroNoiseRootPSD, numMeasurements, timestamp, nedPosition,
                    SMALL_MAGNETOMETER_NOISE_STD));

            // configure calibrator and data source
            KnownPositionAndInstantMagnetometerCalibrator calibrator =
                    new KnownPositionAndInstantMagnetometerCalibrator();
            calibrator.setPosition(nedPosition);
            calibrator.setCommonAxisUsed(true);
            calibrator.setTime(timestamp);

            // create optimizer
            final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                    new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(
                            mDataSource, calibrator);
            optimizer.setListener(this);

            reset();
            assertEquals(0, mStart);
            assertEquals(0, mEnd);

            final double thresholdFactor = optimizer.optimize();

            // check optimization results
            assertEquals(1, mStart);
            assertEquals(1, mEnd);
            assertEquals(thresholdFactor, optimizer.getOptimalThresholdFactor(),
                    0.0);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevel() > 0.0);
            final Acceleration accelerometerBaseNoiseLevel1 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
            assertEquals(accelerometerBaseNoiseLevel1.getValue().doubleValue(),
                    optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                    accelerometerBaseNoiseLevel1.getUnit());
            final Acceleration accelerometerBaseNoiseLevel2 = new Acceleration(
                    1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(
                    accelerometerBaseNoiseLevel2);
            assertEquals(accelerometerBaseNoiseLevel1, accelerometerBaseNoiseLevel2);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getAccelerometerBaseNoiseLevelPsd()),
                    optimizer.getAccelerometerBaseNoiseLevelRootPsd(),
                    ABSOLUTE_ERROR);
            assertTrue(optimizer.getThreshold() > 0.0);
            final Acceleration threshold1 = optimizer.getThresholdAsMeasurement();
            assertEquals(optimizer.getThreshold(), threshold1.getValue().doubleValue(),
                    0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
            final Acceleration threshold2 = new Acceleration(
                    1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getThresholdAsMeasurement(threshold2);
            assertEquals(threshold1, threshold2);
            assertNotNull(optimizer.getEstimatedHardIron());

            final Matrix optimalHardIron = Matrix.newFromArray(optimizer.getEstimatedHardIron());
            final Matrix optimalMm = optimizer.getEstimatedMm();

            assertNotNull(optimalHardIron);
            assertNotNull(optimalMm);

            if (!hardIron.equals(optimalHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(optimalMm, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(optimalHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(optimalMm, LARGE_ABSOLUTE_ERROR));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            final MagnetometerMeasurementsGenerator generator =
                    new MagnetometerMeasurementsGenerator(mGeneratorListener);

            generator.setThresholdFactor(thresholdFactor);

            for (BodyKinematicsAndMagneticFluxDensity bodyKinematics : mBodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(bodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            calibrator.setMeasurements(mGeneratorMeasurements);

            // calibrate
            calibrator.calibrate();

            // check calibration result
            final Matrix estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
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
    public void testOptimizeRobustCalibrator() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            IntervalDetectorThresholdFactorOptimizerException, CalibrationException, IOException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mBodyKinematicsAndMagneticFluxDensities.clear();
            mGeneratorMeasurements.clear();

            // generate measurements

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final int numMeasurements = 3 * KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronGeneral();
            assertNotNull(mm);
            final Date timestamp = new Date(createTimestamp(randomizer));
            NEDPosition nedPosition = createPosition(randomizer);
            assertTrue(generateBodyKinematicsAndMagneticFluxDensity(
                    false, hardIron, mm, accelNoiseRootPSD,
                    gyroNoiseRootPSD, numMeasurements, timestamp, nedPosition,
                    MAGNETOMETER_NOISE_STD));

            // configure calibrator and data source
            PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator calibrator =
                    new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator();
            calibrator.setPosition(nedPosition);
            calibrator.setCommonAxisUsed(false);
            calibrator.setTime(timestamp);

            // create optimizer
            final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                    new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(
                            mDataSource, calibrator);
            optimizer.setListener(this);

            reset();
            assertEquals(0, mStart);
            assertEquals(0, mEnd);

            final double thresholdFactor = optimizer.optimize();

            // check optimization results
            assertEquals(1, mStart);
            assertEquals(1, mEnd);
            assertEquals(thresholdFactor, optimizer.getOptimalThresholdFactor(),
                    0.0);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevel() > 0.0);
            final Acceleration accelerometerBaseNoiseLevel1 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
            assertEquals(accelerometerBaseNoiseLevel1.getValue().doubleValue(),
                    optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                    accelerometerBaseNoiseLevel1.getUnit());
            final Acceleration accelerometerBaseNoiseLevel2 = new Acceleration(
                    1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(
                    accelerometerBaseNoiseLevel2);
            assertEquals(accelerometerBaseNoiseLevel1, accelerometerBaseNoiseLevel2);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getAccelerometerBaseNoiseLevelPsd()),
                    optimizer.getAccelerometerBaseNoiseLevelRootPsd(),
                    ABSOLUTE_ERROR);
            assertTrue(optimizer.getThreshold() > 0.0);
            final Acceleration threshold1 = optimizer.getThresholdAsMeasurement();
            assertEquals(optimizer.getThreshold(), threshold1.getValue().doubleValue(),
                    0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
            final Acceleration threshold2 = new Acceleration(
                    1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getThresholdAsMeasurement(threshold2);
            assertEquals(threshold1, threshold2);
            assertNotNull(optimizer.getEstimatedHardIron());

            final Matrix optimalHardIron = Matrix.newFromArray(optimizer.getEstimatedHardIron());
            final Matrix optimalMm = optimizer.getEstimatedMm();

            assertNotNull(optimalHardIron);
            assertNotNull(optimalMm);

            if (!hardIron.equals(optimalHardIron, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(optimalMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(optimalHardIron, LARGE_ABSOLUTE_ERROR));
            assertTrue(mm.equals(optimalMm, VERY_LARGE_ABSOLUTE_ERROR));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            final MagnetometerMeasurementsGenerator generator =
                    new MagnetometerMeasurementsGenerator(mGeneratorListener);

            generator.setThresholdFactor(thresholdFactor);

            for (BodyKinematicsAndMagneticFluxDensity bodyKinematics : mBodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(bodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            calibrator.setMeasurements(mGeneratorMeasurements);

            // calibrate
            calibrator.calibrate();

            // check calibration result
            final Matrix estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
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

    @Override
    public void onOptimizeStart(
            final IntervalDetectorThresholdFactorOptimizer<BodyKinematicsAndMagneticFluxDensity,
                    MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource> optimizer) {
        mStart++;
        checkLocked((BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer) optimizer);
    }

    @Override
    public void onOptimizeEnd(
            final IntervalDetectorThresholdFactorOptimizer<BodyKinematicsAndMagneticFluxDensity,
                    MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource> optimizer) {
        mEnd++;
        checkLocked((BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer) optimizer);
    }

    private void checkLocked(
            final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer) {
        assertTrue(optimizer.isRunning());
        try {
            optimizer.setDataSource(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            optimizer.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            optimizer.setCalibrator(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            optimizer.setQualityScoreMapper(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            optimizer.setThresholdFactorRange(0.0, 1.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            optimizer.setTimeInterval(0.01);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            optimizer.setTimeInterval(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            optimizer.setMinStaticSamples(0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            optimizer.setMaxDynamicSamples(0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            optimizer.setWindowSize(0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            optimizer.setInitialStaticSamples(0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            optimizer.setInstantaneousNoiseLevelFactor(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            optimizer.setBaseNoiseLevelAbsoluteThreshold(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            optimizer.setBaseNoiseLevelAbsoluteThreshold(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            optimizer.setMseOptimizer(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            optimizer.optimize();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final NavigationException e) {
            fail("LockedException expected but not thrown");
        }
    }

    private void reset() {
        mStart = 0;
        mEnd = 0;
    }

    private boolean generateBodyKinematicsAndMagneticFluxDensity(
            final boolean changePosition, Matrix hardIron, final Matrix mm,
            final double accelNoiseRootPSD, final double gyroNoiseRootPSD,
            final int numMeasurements, final Date timestamp,
            final NEDPosition nedPosition,
            final double magnetometerNoiseStd) throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            IOException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, magnetometerNoiseStd);

        CoordinateTransformation cnb = generateBodyC(randomizer);
        CoordinateTransformation nedC = cnb.inverseAndReturnNew();

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

        final MagnetometerMeasurementsGenerator generator =
                new MagnetometerMeasurementsGenerator();

        // generate initial static samples
        final int initialStaticSamples = TriadStaticIntervalDetector
                .DEFAULT_INITIAL_STATIC_SAMPLES;
        generateStaticSamples(generator, initialStaticSamples, trueKinematics,
                errors, hardIron, mm, wmmEstimator, random, timestamp, nedPosition,
                cnb, noiseRandomizer);

        final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
        final int dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

        for (int i = 0; i < numMeasurements; i++) {
            nedFrame.getPosition(nedPosition);
            nedC = nedFrame.getCoordinateTransformation();
            cnb = nedC.inverseAndReturnNew();

            // generate static samples
            generateStaticSamples(generator, staticPeriodLength, trueKinematics,
                    errors, hardIron, mm, wmmEstimator, random, timestamp, nedPosition,
                    cnb, noiseRandomizer);

            // generate dynamic samples
            generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics,
                    randomizer, ecefFrame, nedFrame, errors, hardIron, mm,
                    wmmEstimator, random, timestamp, nedPosition, cnb,
                    noiseRandomizer, changePosition);
        }

        return generator.getStatus() != TriadStaticIntervalDetector.Status.FAILED;
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

        for (int i = 0; i < numSamples; i++) {
            final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                    .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                            random);

            final BodyMagneticFluxDensity b = generateB(hardIron.getBuffer(),
                    mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition,
                    cnb);
            final BodyKinematicsAndMagneticFluxDensity kb =
                    new BodyKinematicsAndMagneticFluxDensity(
                            measuredKinematics, b);
            assertTrue(generator.process(kb));

            mBodyKinematicsAndMagneticFluxDensities.add(kb);
        }
    }

    @SuppressWarnings("SameParameterValue")
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
            final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                    .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                            random);

            final BodyMagneticFluxDensity b = generateB(hardIron.getBuffer(),
                    mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition,
                    cnb);
            final BodyKinematicsAndMagneticFluxDensity kb =
                    new BodyKinematicsAndMagneticFluxDensity(
                            measuredKinematics, b);
            assertTrue(generator.process(kb));

            mBodyKinematicsAndMagneticFluxDensities.add(kb);

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
