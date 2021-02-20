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
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.ECEFGravity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownFrameAccelerometerNonLinearLeastSquaresCalibrator;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownGravityNormAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.accelerometer.PROMedSRobustKnownGravityNormAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerMeasurementsGeneratorListener;
import com.irurueta.navigation.inertial.calibration.generators.MeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.noise.WindowedTriadNoiseEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
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
import static org.mockito.Mockito.mock;

public class ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizerTest implements
        IntervalDetectorThresholdFactorOptimizerListener<BodyKinematics,
                AccelerometerIntervalDetectorThresholdFactorOptimizerDataSource> {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;
    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;
    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double MIN_DELTA_POS_METERS = -1e-3;
    private static final double MAX_DELTA_POS_METERS = 1e-3;
    private static final double MIN_DELTA_ANGLE_DEGREES = -2.0;
    private static final double MAX_DELTA_ANGLE_DEGREES = 2.0;

    private static final int TIMES = 100;

    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double SMALL_ABSOLUTE_ERROR = 1e-8;

    private static final double SMALL_ROOT_PSD = 1e-15;

    private final List<BodyKinematics> mBodyKinematics = new ArrayList<>();

    private final AccelerometerIntervalDetectorThresholdFactorOptimizerDataSource mDataSource =
            new AccelerometerIntervalDetectorThresholdFactorOptimizerDataSource() {
                @Override
                public int count() {
                    return mBodyKinematics.size();
                }

                @Override
                public BodyKinematics getAt(final int index) {
                    return mBodyKinematics.get(index);
                }
            };

    private final AccelerometerMeasurementsGeneratorListener mGeneratorListener =
            new AccelerometerMeasurementsGeneratorListener() {
                @Override
                public void onInitializationStarted(
                        final AccelerometerMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onInitializationCompleted(
                        final AccelerometerMeasurementsGenerator generator,
                        final double baseNoiseLevel) {
                    // not used
                }

                @Override
                public void onError(
                        final AccelerometerMeasurementsGenerator generator,
                        final TriadStaticIntervalDetector.ErrorReason reason) {
                    // not used
                }

                @Override
                public void onStaticIntervalDetected(
                        final AccelerometerMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onDynamicIntervalDetected(
                        final AccelerometerMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onStaticIntervalSkipped(
                        final AccelerometerMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onDynamicIntervalSkipped(
                        final AccelerometerMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onGeneratedMeasurement(
                        final AccelerometerMeasurementsGenerator generator,
                        final StandardDeviationBodyKinematics measurement) {
                    mGeneratorMeasurements.add(measurement);
                }

                @Override
                public void onReset(
                        final AccelerometerMeasurementsGenerator generator) {
                    // not used
                }
            };

    private final List<StandardDeviationBodyKinematics> mGeneratorMeasurements =
            new ArrayList<>();

    private int mStart;

    private int mEnd;

    private float mProgress;

    @Test
    public void testConstructor1() {
        final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer();

        // check default values
        assertEquals(ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_STEP,
                optimizer.getThresholdFactorStep(), 0.0);
        assertNull(optimizer.getCalibrator());
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultAccelerometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());
        assertEquals(AccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(AccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
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
        assertNull(optimizer.getEstimatedBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedBiasFxVariance());
        assertNull(optimizer.getEstimatedBiasFyVariance());
        assertNull(optimizer.getEstimatedBiasFzVariance());
        assertNull(optimizer.getEstimatedBiases());
        assertNull(optimizer.getEstimatedMa());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA,
                optimizer.getProgressDelta(), 0.0);
    }

    @Test
    public void testConstructor2() {
        final AccelerometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
                mock(AccelerometerIntervalDetectorThresholdFactorOptimizerDataSource.class);

        final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer(dataSource);

        // check default values
        assertEquals(ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_STEP,
                optimizer.getThresholdFactorStep(), 0.0);
        assertNull(optimizer.getCalibrator());
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultAccelerometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());
        assertEquals(AccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(AccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
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
        assertNull(optimizer.getEstimatedBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedBiasFxVariance());
        assertNull(optimizer.getEstimatedBiasFyVariance());
        assertNull(optimizer.getEstimatedBiasFzVariance());
        assertNull(optimizer.getEstimatedBiases());
        assertNull(optimizer.getEstimatedMa());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA,
                optimizer.getProgressDelta(), 0.0);
    }

    @Test
    public void testConstructor3() {
        final KnownGravityNormAccelerometerCalibrator calibrator =
                new KnownGravityNormAccelerometerCalibrator();

        ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer(
                        calibrator);

        // check default values
        assertEquals(ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_STEP,
                optimizer.getThresholdFactorStep(), 0.0);
        assertSame(calibrator, optimizer.getCalibrator());
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultAccelerometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());
        assertEquals(AccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(AccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
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
        assertNull(optimizer.getEstimatedBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedBiasFxVariance());
        assertNull(optimizer.getEstimatedBiasFyVariance());
        assertNull(optimizer.getEstimatedBiasFzVariance());
        assertNull(optimizer.getEstimatedBiases());
        assertNull(optimizer.getEstimatedMa());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA,
                optimizer.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        optimizer = null;
        try {
            optimizer = new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer(
                    new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(optimizer);
    }

    @Test
    public void testConstructor4() {
        final AccelerometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
                mock(AccelerometerIntervalDetectorThresholdFactorOptimizerDataSource.class);
        final KnownGravityNormAccelerometerCalibrator calibrator =
                new KnownGravityNormAccelerometerCalibrator();

        ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer(
                        dataSource, calibrator);

        // check default values
        assertEquals(ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_STEP,
                optimizer.getThresholdFactorStep(), 0.0);
        assertSame(calibrator, optimizer.getCalibrator());
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultAccelerometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());
        assertEquals(AccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(AccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
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
        assertNull(optimizer.getEstimatedBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedBiasFxVariance());
        assertNull(optimizer.getEstimatedBiasFyVariance());
        assertNull(optimizer.getEstimatedBiasFzVariance());
        assertNull(optimizer.getEstimatedBiases());
        assertNull(optimizer.getEstimatedMa());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA,
                optimizer.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        optimizer = null;
        try {
            optimizer = new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer(
                    dataSource, new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(optimizer);
    }

    @Test
    public void testGetSetThresholdFactorStep() throws LockedException {
        final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_STEP,
                optimizer.getThresholdFactorStep(), 0.0);

        // set new value
        optimizer.setThresholdFactorStep(2.0);

        // check
        assertEquals(2.0, optimizer.getThresholdFactorStep(), 0.0);

        // Force IllegalArgumentException
        try {
            optimizer.setThresholdFactorStep(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetCalibrator() throws LockedException {
        final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNull(optimizer.getCalibrator());

        // set new value
        final KnownGravityNormAccelerometerCalibrator calibrator =
                new KnownGravityNormAccelerometerCalibrator();

        optimizer.setCalibrator(calibrator);

        // check
        assertSame(calibrator, optimizer.getCalibrator());

        // Force IllegalArgumentException
        try {
            optimizer.setCalibrator(
                    new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetQualityScoreMapper() throws LockedException {
        final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultAccelerometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());

        // set new value
        //noinspection unchecked
        final QualityScoreMapper<StandardDeviationBodyKinematics> qualityScoreMapper =
                mock(QualityScoreMapper.class);
        optimizer.setQualityScoreMapper(qualityScoreMapper);

        // check
        assertSame(qualityScoreMapper, optimizer.getQualityScoreMapper());
    }

    @Test
    public void testGetSetThresholdFactorRange() throws LockedException {
        final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer();

        // check default values
        assertEquals(AccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(AccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
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
        final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer();

        // check default values
        assertNull(optimizer.getDataSource());

        // set new value
        final AccelerometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
                mock(AccelerometerIntervalDetectorThresholdFactorOptimizerDataSource.class);

        optimizer.setDataSource(dataSource);

        // check
        assertSame(dataSource, optimizer.getDataSource());
    }

    @Test
    public void testIsReady() throws LockedException {
        final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertFalse(optimizer.isReady());

        // set data source
        final AccelerometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
                mock(AccelerometerIntervalDetectorThresholdFactorOptimizerDataSource.class);
        optimizer.setDataSource(dataSource);

        // check
        assertFalse(optimizer.isReady());

        // set calibrator
        final KnownGravityNormAccelerometerCalibrator calibrator =
                new KnownGravityNormAccelerometerCalibrator();
        optimizer.setCalibrator(calibrator);

        // check
        assertTrue(optimizer.isReady());

        // unset quality score mapper
        optimizer.setQualityScoreMapper(null);

        // check
        assertFalse(optimizer.isReady());
    }

    @Test
    public void testGetSetTimeInterval() throws LockedException {
        final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer();

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
        final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer();

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
        final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer();

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
        final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer();

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
        final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer();

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
        final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer();

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
        final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer();

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
        final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer();

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
        final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer();

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
        final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNull(optimizer.getListener());

        // set new value
        optimizer.setListener(this);

        // check
        assertSame(this, optimizer.getListener());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer();

        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA,
                optimizer.getProgressDelta(), 0.0);

        // set new value
        optimizer.setProgressDelta(0.5f);

        // check
        assertEquals(0.5f, optimizer.getProgressDelta(), 0.0f);

        // Force IllegalArgumentException
        try {
            optimizer.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            optimizer.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testOptimizeGeneralWithNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            IntervalDetectorThresholdFactorOptimizerException, CalibrationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mBodyKinematics.clear();
            mGeneratorMeasurements.clear();

            // generate measurements

            final NEDFrame nedFrame = generateFrame();
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            final ECEFGravity gravity = ECEFGravityEstimator
                    .estimateGravityAndReturnNew(ecefFrame);

            final Matrix ma = generateMaGeneral();
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            assertTrue(generateBodyKinematics(nedFrame, ecefFrame, false, ma,
                    accelNoiseRootPSD, gyroNoiseRootPSD, numMeasurements));

            // configure calibrator and data source
            final Matrix initialBa = new Matrix(3, 1);
            final Matrix initialMa = new Matrix(3, 3);
            final KnownGravityNormAccelerometerCalibrator calibrator =
                    new KnownGravityNormAccelerometerCalibrator(
                            gravity.getNorm(), false, initialBa,
                            initialMa);

            // create optimizer
            final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                    new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer(
                            mDataSource, calibrator);
            optimizer.setListener(this);

            reset();
            assertEquals(0, mStart);
            assertEquals(0, mEnd);
            assertEquals(0.0f, mProgress, 0.0f);

            final double thresholdFactor = optimizer.optimize();

            // check optimization results
            assertEquals(1, mStart);
            assertEquals(1, mEnd);
            assertTrue(mProgress > 0.0f);
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
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getAccelerometerBaseNoiseLevelPsd()),
                    optimizer.getAccelerometerBaseNoiseLevelRootPsd(),
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getThreshold() > 0.0);
            final Acceleration threshold1 = optimizer.getThresholdAsMeasurement();
            assertEquals(optimizer.getThreshold(), threshold1.getValue().doubleValue(),
                    0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
            final Acceleration threshold2 = new Acceleration(
                    1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getThresholdAsMeasurement(threshold2);
            assertEquals(threshold1, threshold2);
            assertTrue(optimizer.getEstimatedBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedBiasFxVariance() > 0.0);
            assertTrue(optimizer.getEstimatedBiasFyVariance() > 0.0);
            assertTrue(optimizer.getEstimatedBiasFzVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedBiases());

            final Matrix optimalBa = Matrix.newFromArray(optimizer.getEstimatedBiases());
            final Matrix optimalMa = optimizer.getEstimatedMa();

            assertNotNull(optimalBa);
            assertNotNull(optimalMa);

            final Matrix ba = generateBa();

            if (!ba.equals(optimalBa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(optimalMa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(optimalBa, LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(optimalMa, LARGE_ABSOLUTE_ERROR));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            final AccelerometerMeasurementsGenerator generator =
                    new AccelerometerMeasurementsGenerator(mGeneratorListener);

            generator.setThresholdFactor(thresholdFactor);

            for (BodyKinematics bodyKinematics : mBodyKinematics) {
                assertTrue(generator.process(bodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            calibrator.setMeasurements(mGeneratorMeasurements);

            // calibrate
            calibrator.calibrate();

            // check calibration result
            final Matrix estimatedBa = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = calibrator.getEstimatedMa();

            if (!ba.equals(estimatedBa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testOptimizeCommonAxisSmallNoiseOnlyRotation()
            throws InvalidSourceAndDestinationFrameTypeException, WrongSizeException,
            LockedException, NotReadyException, IntervalDetectorThresholdFactorOptimizerException,
            CalibrationException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mBodyKinematics.clear();
            mGeneratorMeasurements.clear();

            // generate measurements

            final NEDFrame nedFrame = generateFrame();
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            final ECEFGravity gravity = ECEFGravityEstimator
                    .estimateGravityAndReturnNew(ecefFrame);

            final Matrix ma = generateMaCommonAxis();
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            assertTrue(generateBodyKinematics(nedFrame, ecefFrame, false, ma,
                    SMALL_ROOT_PSD, SMALL_ROOT_PSD, numMeasurements));

            // configure calibrator and data source
            final Matrix initialBa = new Matrix(3, 1);
            final Matrix initialMa = new Matrix(3, 3);
            final KnownGravityNormAccelerometerCalibrator calibrator =
                    new KnownGravityNormAccelerometerCalibrator(
                            gravity.getNorm(), true, initialBa,
                            initialMa);

            // create optimizer
            final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                    new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer(
                            mDataSource, calibrator);
            optimizer.setListener(this);

            reset();
            assertEquals(0, mStart);
            assertEquals(0, mEnd);
            assertEquals(0.0f, mProgress, 0.0f);

            final double thresholdFactor = optimizer.optimize();

            // check optimization results
            assertEquals(1, mStart);
            assertEquals(1, mEnd);
            assertTrue(mProgress > 0.0f);
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
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getAccelerometerBaseNoiseLevelPsd()),
                    optimizer.getAccelerometerBaseNoiseLevelRootPsd(),
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getThreshold() > 0.0);
            final Acceleration threshold1 = optimizer.getThresholdAsMeasurement();
            assertEquals(optimizer.getThreshold(), threshold1.getValue().doubleValue(),
                    0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
            final Acceleration threshold2 = new Acceleration(
                    1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getThresholdAsMeasurement(threshold2);
            assertEquals(threshold1, threshold2);
            assertTrue(optimizer.getEstimatedBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedBiasFxVariance() > 0.0);
            assertTrue(optimizer.getEstimatedBiasFyVariance() > 0.0);
            assertTrue(optimizer.getEstimatedBiasFzVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedBiases());

            final Matrix optimalBa = Matrix.newFromArray(optimizer.getEstimatedBiases());
            final Matrix optimalMa = optimizer.getEstimatedMa();

            assertNotNull(optimalBa);
            assertNotNull(optimalMa);

            final Matrix ba = generateBa();

            if (!ba.equals(optimalBa, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(optimalMa, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(optimalBa, SMALL_ABSOLUTE_ERROR));
            assertTrue(ma.equals(optimalMa, SMALL_ABSOLUTE_ERROR));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            final AccelerometerMeasurementsGenerator generator =
                    new AccelerometerMeasurementsGenerator(mGeneratorListener);

            generator.setThresholdFactor(thresholdFactor);

            for (BodyKinematics bodyKinematics : mBodyKinematics) {
                assertTrue(generator.process(bodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            calibrator.setMeasurements(mGeneratorMeasurements);

            // calibrate
            calibrator.calibrate();

            // check calibration result
            final Matrix estimatedBa = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = calibrator.getEstimatedMa();

            if (!ba.equals(estimatedBa, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, SMALL_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, SMALL_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testOptimizeCommonAxisSmallNoiseWithRotationAndPositionChange()
            throws InvalidSourceAndDestinationFrameTypeException, WrongSizeException,
            LockedException, NotReadyException, IntervalDetectorThresholdFactorOptimizerException,
            CalibrationException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mBodyKinematics.clear();
            mGeneratorMeasurements.clear();

            // generate measurements

            final NEDFrame nedFrame = generateFrame();
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            final ECEFGravity gravity = ECEFGravityEstimator
                    .estimateGravityAndReturnNew(ecefFrame);

            final Matrix ma = generateMaCommonAxis();
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            assertTrue(generateBodyKinematics(nedFrame, ecefFrame, true, ma,
                    SMALL_ROOT_PSD, SMALL_ROOT_PSD, numMeasurements));

            // configure calibrator and data source
            final Matrix initialBa = new Matrix(3, 1);
            final Matrix initialMa = new Matrix(3, 3);
            final KnownGravityNormAccelerometerCalibrator calibrator =
                    new KnownGravityNormAccelerometerCalibrator(
                            gravity.getNorm(), true, initialBa,
                            initialMa);

            // create optimizer
            final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                    new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer(
                            mDataSource, calibrator);
            optimizer.setListener(this);

            reset();
            assertEquals(0, mStart);
            assertEquals(0, mEnd);
            assertEquals(0.0f, mProgress, 0.0f);

            final double thresholdFactor = optimizer.optimize();

            // check optimization results
            assertEquals(1, mStart);
            assertEquals(1, mEnd);
            assertTrue(mProgress > 0.0f);
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
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getAccelerometerBaseNoiseLevelPsd()),
                    optimizer.getAccelerometerBaseNoiseLevelRootPsd(),
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getThreshold() > 0.0);
            final Acceleration threshold1 = optimizer.getThresholdAsMeasurement();
            assertEquals(optimizer.getThreshold(), threshold1.getValue().doubleValue(),
                    0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
            final Acceleration threshold2 = new Acceleration(
                    1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getThresholdAsMeasurement(threshold2);
            assertEquals(threshold1, threshold2);
            assertTrue(optimizer.getEstimatedBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedBiasFxVariance() > 0.0);
            assertTrue(optimizer.getEstimatedBiasFyVariance() > 0.0);
            assertTrue(optimizer.getEstimatedBiasFzVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedBiases());

            final Matrix optimalBa = Matrix.newFromArray(optimizer.getEstimatedBiases());
            final Matrix optimalMa = optimizer.getEstimatedMa();

            assertNotNull(optimalBa);
            assertNotNull(optimalMa);

            final Matrix ba = generateBa();

            if (!ba.equals(optimalBa, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(optimalMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(optimalBa, ABSOLUTE_ERROR));
            assertTrue(ma.equals(optimalMa, ABSOLUTE_ERROR));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            final AccelerometerMeasurementsGenerator generator =
                    new AccelerometerMeasurementsGenerator(mGeneratorListener);

            generator.setThresholdFactor(thresholdFactor);

            for (BodyKinematics bodyKinematics : mBodyKinematics) {
                assertTrue(generator.process(bodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            calibrator.setMeasurements(mGeneratorMeasurements);

            // calibrate
            calibrator.calibrate();

            // check calibration result
            final Matrix estimatedBa = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = calibrator.getEstimatedMa();

            if (!ba.equals(estimatedBa, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testOptimizeRobustCalibrator() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            IntervalDetectorThresholdFactorOptimizerException, CalibrationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mBodyKinematics.clear();
            mGeneratorMeasurements.clear();

            // generate measurements

            final NEDFrame nedFrame = generateFrame();
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            final ECEFGravity gravity = ECEFGravityEstimator
                    .estimateGravityAndReturnNew(ecefFrame);

            final Matrix ma = generateMaGeneral();
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final int numMeasurements = 3 * KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            assertTrue(generateBodyKinematics(nedFrame, ecefFrame, false, ma,
                    accelNoiseRootPSD, gyroNoiseRootPSD, numMeasurements));

            // configure calibrator and data source
            final Matrix initialBa = new Matrix(3, 1);
            final Matrix initialMa = new Matrix(3, 3);
            final PROMedSRobustKnownGravityNormAccelerometerCalibrator calibrator =
                    new PROMedSRobustKnownGravityNormAccelerometerCalibrator();
            calibrator.setGroundTruthGravityNorm(gravity.getNorm());
            calibrator.setCommonAxisUsed(false);
            calibrator.setInitialBias(initialBa);
            calibrator.setInitialMa(initialMa);

            // create optimizer
            final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer =
                    new ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer(
                            mDataSource, calibrator);
            optimizer.setListener(this);

            reset();
            assertEquals(0, mStart);
            assertEquals(0, mEnd);
            assertEquals(0.0f, mProgress, 0.0f);

            final double thresholdFactor = optimizer.optimize();

            // check optimization results
            assertEquals(1, mStart);
            assertEquals(1, mEnd);
            assertTrue(mProgress > 0.0f);
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
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getAccelerometerBaseNoiseLevelPsd()),
                    optimizer.getAccelerometerBaseNoiseLevelRootPsd(),
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getThreshold() > 0.0);
            final Acceleration threshold1 = optimizer.getThresholdAsMeasurement();
            assertEquals(optimizer.getThreshold(), threshold1.getValue().doubleValue(),
                    0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
            final Acceleration threshold2 = new Acceleration(
                    1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getThresholdAsMeasurement(threshold2);
            assertEquals(threshold1, threshold2);
            assertTrue(optimizer.getEstimatedBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedBiasFxVariance() > 0.0);
            assertTrue(optimizer.getEstimatedBiasFyVariance() > 0.0);
            assertTrue(optimizer.getEstimatedBiasFzVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedBiases());

            final Matrix optimalBa = Matrix.newFromArray(optimizer.getEstimatedBiases());
            final Matrix optimalMa = optimizer.getEstimatedMa();

            assertNotNull(optimalBa);
            assertNotNull(optimalMa);

            final Matrix ba = generateBa();

            if (!ba.equals(optimalBa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(optimalMa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(optimalBa, LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(optimalMa, LARGE_ABSOLUTE_ERROR));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            final AccelerometerMeasurementsGenerator generator =
                    new AccelerometerMeasurementsGenerator(mGeneratorListener);

            generator.setThresholdFactor(thresholdFactor);

            for (BodyKinematics bodyKinematics : mBodyKinematics) {
                assertTrue(generator.process(bodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            calibrator.setMeasurements(mGeneratorMeasurements);

            // calibrate
            calibrator.calibrate();

            // check calibration result
            final Matrix estimatedBa = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = calibrator.getEstimatedMa();

            if (!ba.equals(estimatedBa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onOptimizeStart(
            final IntervalDetectorThresholdFactorOptimizer<BodyKinematics,
                    AccelerometerIntervalDetectorThresholdFactorOptimizerDataSource> optimizer) {
        mStart++;
        checkLocked((ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer) optimizer);
    }

    @Override
    public void onOptimizeEnd(
            final IntervalDetectorThresholdFactorOptimizer<BodyKinematics,
                    AccelerometerIntervalDetectorThresholdFactorOptimizerDataSource> optimizer) {
        mEnd++;
        checkLocked((ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer) optimizer);
    }

    @Override
    public void onOptimizeProgressChange(
            final IntervalDetectorThresholdFactorOptimizer<BodyKinematics,
                    AccelerometerIntervalDetectorThresholdFactorOptimizerDataSource> optimizer,
            final float progress) {
        assertTrue(progress >= 0.0f);
        assertTrue(progress <= 1.0f);
        assertTrue(progress > mProgress);
        if (mProgress == 0.0f) {
            checkLocked((ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer) optimizer);
        }
        mProgress = progress;
    }

    private void checkLocked(
            final ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer optimizer) {
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
            optimizer.setThresholdFactorStep(0.0);
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

    private boolean generateBodyKinematics(
            final NEDFrame nedFrame, final ECEFFrame ecefFrame,
            final boolean changePosition, final Matrix ma, final double accelNoiseRootPSD,
            final double gyroNoiseRootPSD, final int numMeasurements)
            throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

        final AccelerometerMeasurementsGenerator generator =
                new AccelerometerMeasurementsGenerator();

        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);

        // generate initial static samples
        final int initialStaticSamples = TriadStaticIntervalDetector
                .DEFAULT_INITIAL_STATIC_SAMPLES;
        generateStaticSamples(generator, initialStaticSamples, trueKinematics,
                errors, random);

        final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
        final int dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

        for (int i = 0; i < numMeasurements; i++) {
            // generate static samples
            generateStaticSamples(generator, staticPeriodLength, trueKinematics,
                    errors, random);

            // generate dynamic samples
            generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics,
                    randomizer, ecefFrame, nedFrame, errors, random, changePosition);
        }

        return generator.getStatus() != TriadStaticIntervalDetector.Status.FAILED;
    }

    private NEDFrame generateFrame() throws InvalidSourceAndDestinationFrameTypeException {
        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final CoordinateTransformation nedC = new CoordinateTransformation(
                roll, pitch, yaw, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        return new NEDFrame(nedPosition, nedC);
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

    private Matrix generateMaGeneral() throws WrongSizeException {
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

    private void generateStaticSamples(
            final AccelerometerMeasurementsGenerator generator,
            final int numSamples,
            final BodyKinematics trueKinematics,
            final IMUErrors errors,
            final Random random)
            throws LockedException {

        for (int i = 0; i < numSamples; i++) {
            final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                    .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                            random);

            assertTrue(generator.process(measuredKinematics));

            mBodyKinematics.add(measuredKinematics);
        }
    }

    @SuppressWarnings("SameParameterValue")
    private void generateDynamicSamples(
            final AccelerometerMeasurementsGenerator generator,
            final int numSamples,
            final BodyKinematics trueKinematics,
            final UniformRandomizer randomizer,
            final ECEFFrame ecefFrame,
            final NEDFrame nedFrame,
            final IMUErrors errors,
            final Random random,
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

            assertTrue(generator.process(measuredKinematics));

            mBodyKinematics.add(measuredKinematics);

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

    private void reset() {
        mStart = 0;
        mEnd = 0;
        mProgress = 0.0f;
    }
}
