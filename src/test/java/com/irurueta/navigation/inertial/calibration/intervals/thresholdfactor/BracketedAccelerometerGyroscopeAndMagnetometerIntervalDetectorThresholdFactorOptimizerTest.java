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
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
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
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.ECEFGravity;
import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.calibration.*;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownFrameAccelerometerNonLinearLeastSquaresCalibrator;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownGravityNormAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.accelerometer.PROMedSRobustKnownGravityNormAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener;
import com.irurueta.navigation.inertial.calibration.generators.MeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.gyroscope.EasyGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.KnownFrameGyroscopeNonLinearLeastSquaresCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.PROMedSRobustEasyGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionIntegrator;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.magnetometer.KnownFrameMagnetometerNonLinearLeastSquaresCalibrator;
import com.irurueta.navigation.inertial.calibration.magnetometer.KnownPositionAndInstantMagnetometerCalibrator;
import com.irurueta.navigation.inertial.calibration.magnetometer.PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator;
import com.irurueta.navigation.inertial.calibration.noise.WindowedTriadNoiseEstimator;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
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

public class BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerTest implements
        IntervalDetectorThresholdFactorOptimizerListener<TimedBodyKinematicsAndMagneticFluxDensity,
                AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource> {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double MIN_HARD_IRON = -1e-5;
    private static final double MAX_HARD_IRON = 1e-5;

    private static final double MIN_SOFT_IRON = -1e-6;
    private static final double MAX_SOFT_IRON = 1e-6;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;
    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;
    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double SMALL_MAGNETOMETER_NOISE_STD = 1e-12;
    private static final double MAGNETOMETER_NOISE_STD = 200e-9;

    private static final double MIN_DELTA_POS_METERS = -1e-3;
    private static final double MAX_DELTA_POS_METERS = 1e-3;
    private static final double MIN_DELTA_ANGLE_DEGREES = -2.0;
    private static final double MAX_DELTA_ANGLE_DEGREES = 2.0;

    private static final int TIMES = 100;

    private static final double ABSOLUTE_ERROR = 5e-4;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-3;
    private static final double SMALL_ABSOLUTE_ERROR = 1e-6;

    private static final double SMALL_ROOT_PSD = 1e-15;

    private static final double ABSOLUTE_ERROR_MAGNETOMETER = 1e-9;
    private static final double LARGE_ABSOLUTE_ERROR_MAGNETOMETER = 5e-5;
    private static final double VERY_LARGE_ABSOLUTE_ERROR_MAGNETOMETER = 1e-2;

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

    private final List<TimedBodyKinematicsAndMagneticFluxDensity> mTimedBodyKinematicsAndMagneticFluxDensities =
            new ArrayList<>();

    private final AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource mDataSource =
            new AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource() {

                @Override
                public int count() {
                    return mTimedBodyKinematicsAndMagneticFluxDensities.size();
                }

                @Override
                public TimedBodyKinematicsAndMagneticFluxDensity getAt(final int index) {
                    return mTimedBodyKinematicsAndMagneticFluxDensities.get(index);
                }
            };

    private final AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener mGeneratorListener =
            new AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener() {
                @Override
                public void onInitializationStarted(
                        final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onInitializationCompleted(
                        final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
                        final double accelerometerBaseNoiseLevel) {
                    // not used
                }

                @Override
                public void onError(
                        final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
                        final TriadStaticIntervalDetector.ErrorReason reason) {
                    // not used
                }

                @Override
                public void onStaticIntervalDetected(
                        final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onDynamicIntervalDetected(
                        final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onStaticIntervalSkipped(
                        final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onDynamicIntervalSkipped(
                        final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onGeneratedAccelerometerMeasurement(
                        final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
                        final StandardDeviationBodyKinematics measurement) {
                    mAccelerometerGeneratorMeasurements.add(measurement);
                }

                @Override
                public void onGeneratedGyroscopeMeasurement(
                        final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
                        final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence) {
                    mGyroscopeGeneratorMeasurements.add(sequence);
                }

                @Override
                public void onGeneratedMagnetometerMeasurement(
                        final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
                        final StandardDeviationBodyMagneticFluxDensity measurement) {
                    mMagnetometerGeneratorMeasurements.add(measurement);
                }

                @Override
                public void onReset(
                        final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
                    // not used
                }
            };

    private final List<StandardDeviationBodyKinematics> mAccelerometerGeneratorMeasurements =
            new ArrayList<>();

    private final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> mGyroscopeGeneratorMeasurements =
            new ArrayList<>();

    private final List<StandardDeviationBodyMagneticFluxDensity> mMagnetometerGeneratorMeasurements =
            new ArrayList<>();

    private int mStart;

    private int mEnd;

    private float mProgress;

    @Test
    public void testConstructor1() {
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default values
        assertNotNull(optimizer.getMseOptimizer());
        assertNull(optimizer.getAccelerometerCalibrator());
        assertNull(optimizer.getGyroscopeCalibrator());
        assertNull(optimizer.getMagnetometerCalibrator());
        assertNotNull(optimizer.getAccelerometerQualityScoreMapper());
        assertEquals(DefaultAccelerometerQualityScoreMapper.class,
                optimizer.getAccelerometerQualityScoreMapper().getClass());
        assertNotNull(optimizer.getGyroscopeQualityScoreMapper());
        assertEquals(DefaultGyroscopeQualityScoreMapper.class,
                optimizer.getGyroscopeQualityScoreMapper().getClass());
        assertNotNull(optimizer.getMagnetometerQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class,
                optimizer.getMagnetometerQualityScoreMapper().getClass());
        assertNotNull(optimizer.getMseRule());
        assertEquals(DefaultAccelerometerGyroscopeAndMagnetometerMseRule.class,
                optimizer.getMseRule().getClass());
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
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
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelPsd(),
                0.0);
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelRootPsd(),
                0.0);
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
        assertNull(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedAccelerometerBiasFxVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFyVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFzVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiases());
        assertNull(optimizer.getEstimatedAccelerometerMa());
        assertNull(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedGyroscopeBiasXVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasYVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasZVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiases());
        assertNull(optimizer.getEstimatedGyroscopeMg());
        assertNull(optimizer.getEstimatedGyroscopeGg());
        assertNull(optimizer.getEstimatedMagnetometerHardIron());
        assertNull(optimizer.getEstimatedMagnetometerMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA,
                optimizer.getProgressDelta(), 0.0);
    }

    @Test
    public void testConstructor2() {
        final AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
                mock(AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource.class);

        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                        dataSource);

        // check default values
        assertNotNull(optimizer.getMseOptimizer());
        assertNull(optimizer.getAccelerometerCalibrator());
        assertNull(optimizer.getGyroscopeCalibrator());
        assertNull(optimizer.getMagnetometerCalibrator());
        assertNotNull(optimizer.getAccelerometerQualityScoreMapper());
        assertEquals(DefaultAccelerometerQualityScoreMapper.class,
                optimizer.getAccelerometerQualityScoreMapper().getClass());
        assertNotNull(optimizer.getGyroscopeQualityScoreMapper());
        assertEquals(DefaultGyroscopeQualityScoreMapper.class,
                optimizer.getGyroscopeQualityScoreMapper().getClass());
        assertNotNull(optimizer.getMagnetometerQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class,
                optimizer.getMagnetometerQualityScoreMapper().getClass());
        assertNotNull(optimizer.getMseRule());
        assertEquals(DefaultAccelerometerGyroscopeAndMagnetometerMseRule.class,
                optimizer.getMseRule().getClass());
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
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
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelPsd(),
                0.0);
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelRootPsd(),
                0.0);
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
        assertNull(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedAccelerometerBiasFxVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFyVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFzVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiases());
        assertNull(optimizer.getEstimatedAccelerometerMa());
        assertNull(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedGyroscopeBiasXVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasYVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasZVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiases());
        assertNull(optimizer.getEstimatedGyroscopeMg());
        assertNull(optimizer.getEstimatedGyroscopeGg());
        assertNull(optimizer.getEstimatedMagnetometerHardIron());
        assertNull(optimizer.getEstimatedMagnetometerMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA,
                optimizer.getProgressDelta(), 0.0);
    }

    @Test
    public void testConstructor3() {
        final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                new KnownGravityNormAccelerometerCalibrator();
        final EasyGyroscopeCalibrator gyroscopeCalibrator =
                new EasyGyroscopeCalibrator();
        final KnownPositionAndInstantMagnetometerCalibrator magnetometerCalibrator =
                new KnownPositionAndInstantMagnetometerCalibrator();

        BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                        accelerometerCalibrator, gyroscopeCalibrator, magnetometerCalibrator);

        // check default values
        assertNotNull(optimizer.getMseOptimizer());
        assertSame(accelerometerCalibrator, optimizer.getAccelerometerCalibrator());
        assertSame(gyroscopeCalibrator, optimizer.getGyroscopeCalibrator());
        assertSame(magnetometerCalibrator, optimizer.getMagnetometerCalibrator());
        assertNotNull(optimizer.getAccelerometerQualityScoreMapper());
        assertEquals(DefaultAccelerometerQualityScoreMapper.class,
                optimizer.getAccelerometerQualityScoreMapper().getClass());
        assertNotNull(optimizer.getGyroscopeQualityScoreMapper());
        assertEquals(DefaultGyroscopeQualityScoreMapper.class,
                optimizer.getGyroscopeQualityScoreMapper().getClass());
        assertNotNull(optimizer.getMagnetometerQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class,
                optimizer.getMagnetometerQualityScoreMapper().getClass());
        assertNotNull(optimizer.getMseRule());
        assertEquals(DefaultAccelerometerGyroscopeAndMagnetometerMseRule.class,
                optimizer.getMseRule().getClass());
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
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
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelPsd(),
                0.0);
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelRootPsd(),
                0.0);
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
        assertNull(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedAccelerometerBiasFxVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFyVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFzVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiases());
        assertNull(optimizer.getEstimatedAccelerometerMa());
        assertNull(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedGyroscopeBiasXVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasYVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasZVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiases());
        assertNull(optimizer.getEstimatedGyroscopeMg());
        assertNull(optimizer.getEstimatedGyroscopeGg());
        assertNull(optimizer.getEstimatedMagnetometerHardIron());
        assertNull(optimizer.getEstimatedMagnetometerMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA,
                optimizer.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        optimizer = null;
        try {
            optimizer = new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                    new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(),
                    gyroscopeCalibrator, magnetometerCalibrator);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            optimizer = new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                    accelerometerCalibrator, new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(),
                    magnetometerCalibrator);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            optimizer = new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                    accelerometerCalibrator, gyroscopeCalibrator,
                    new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(optimizer);
    }

    @Test
    public void testConstructor4() {
        final AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
                mock(AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource.class);

        final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                new KnownGravityNormAccelerometerCalibrator();
        final EasyGyroscopeCalibrator gyroscopeCalibrator =
                new EasyGyroscopeCalibrator();
        final KnownPositionAndInstantMagnetometerCalibrator magnetometerCalibrator =
                new KnownPositionAndInstantMagnetometerCalibrator();

        BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                        dataSource, accelerometerCalibrator, gyroscopeCalibrator,
                        magnetometerCalibrator);

        // check default values
        assertNotNull(optimizer.getMseOptimizer());
        assertSame(accelerometerCalibrator, optimizer.getAccelerometerCalibrator());
        assertSame(gyroscopeCalibrator, optimizer.getGyroscopeCalibrator());
        assertSame(magnetometerCalibrator, optimizer.getMagnetometerCalibrator());
        assertNotNull(optimizer.getAccelerometerQualityScoreMapper());
        assertEquals(DefaultAccelerometerQualityScoreMapper.class,
                optimizer.getAccelerometerQualityScoreMapper().getClass());
        assertNotNull(optimizer.getGyroscopeQualityScoreMapper());
        assertEquals(DefaultGyroscopeQualityScoreMapper.class,
                optimizer.getGyroscopeQualityScoreMapper().getClass());
        assertNotNull(optimizer.getMagnetometerQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class,
                optimizer.getMagnetometerQualityScoreMapper().getClass());
        assertNotNull(optimizer.getMseRule());
        assertEquals(DefaultAccelerometerGyroscopeAndMagnetometerMseRule.class,
                optimizer.getMseRule().getClass());
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
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
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelPsd(),
                0.0);
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelRootPsd(),
                0.0);
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
        assertNull(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedAccelerometerBiasFxVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFyVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFzVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiases());
        assertNull(optimizer.getEstimatedAccelerometerMa());
        assertNull(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedGyroscopeBiasXVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasYVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasZVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiases());
        assertNull(optimizer.getEstimatedGyroscopeMg());
        assertNull(optimizer.getEstimatedGyroscopeGg());
        assertNull(optimizer.getEstimatedMagnetometerHardIron());
        assertNull(optimizer.getEstimatedMagnetometerMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA,
                optimizer.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        optimizer = null;
        try {
            optimizer = new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                    dataSource, new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(),
                    gyroscopeCalibrator, magnetometerCalibrator);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            optimizer = new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                    dataSource, accelerometerCalibrator,
                    new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(),
                    magnetometerCalibrator);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            optimizer = new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                    dataSource, accelerometerCalibrator, gyroscopeCalibrator,
                    new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(optimizer);
    }

    @Test
    public void testConstructor5() {
        final BrentSingleOptimizer mseOptimizer = new BrentSingleOptimizer();

        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                        mseOptimizer);

        // check default values
        assertSame(mseOptimizer, optimizer.getMseOptimizer());
        assertNull(optimizer.getAccelerometerCalibrator());
        assertNull(optimizer.getGyroscopeCalibrator());
        assertNull(optimizer.getMagnetometerCalibrator());
        assertNotNull(optimizer.getAccelerometerQualityScoreMapper());
        assertEquals(DefaultAccelerometerQualityScoreMapper.class,
                optimizer.getAccelerometerQualityScoreMapper().getClass());
        assertNotNull(optimizer.getGyroscopeQualityScoreMapper());
        assertEquals(DefaultGyroscopeQualityScoreMapper.class,
                optimizer.getGyroscopeQualityScoreMapper().getClass());
        assertNotNull(optimizer.getMagnetometerQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class,
                optimizer.getMagnetometerQualityScoreMapper().getClass());
        assertNotNull(optimizer.getMseRule());
        assertEquals(DefaultAccelerometerGyroscopeAndMagnetometerMseRule.class,
                optimizer.getMseRule().getClass());
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
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
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelPsd(),
                0.0);
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelRootPsd(),
                0.0);
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
        assertNull(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedAccelerometerBiasFxVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFyVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFzVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiases());
        assertNull(optimizer.getEstimatedAccelerometerMa());
        assertNull(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedGyroscopeBiasXVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasYVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasZVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiases());
        assertNull(optimizer.getEstimatedGyroscopeMg());
        assertNull(optimizer.getEstimatedGyroscopeGg());
        assertNull(optimizer.getEstimatedMagnetometerHardIron());
        assertNull(optimizer.getEstimatedMagnetometerMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA,
                optimizer.getProgressDelta(), 0.0);
    }

    @Test
    public void testConstructor6() {
        final AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
                mock(AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource.class);
        final BrentSingleOptimizer mseOptimizer = new BrentSingleOptimizer();

        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                        dataSource, mseOptimizer);

        // check default values
        assertSame(mseOptimizer, optimizer.getMseOptimizer());
        assertNull(optimizer.getAccelerometerCalibrator());
        assertNull(optimizer.getGyroscopeCalibrator());
        assertNull(optimizer.getMagnetometerCalibrator());
        assertNotNull(optimizer.getAccelerometerQualityScoreMapper());
        assertEquals(DefaultAccelerometerQualityScoreMapper.class,
                optimizer.getAccelerometerQualityScoreMapper().getClass());
        assertNotNull(optimizer.getGyroscopeQualityScoreMapper());
        assertEquals(DefaultGyroscopeQualityScoreMapper.class,
                optimizer.getGyroscopeQualityScoreMapper().getClass());
        assertNotNull(optimizer.getMagnetometerQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class,
                optimizer.getMagnetometerQualityScoreMapper().getClass());
        assertNotNull(optimizer.getMseRule());
        assertEquals(DefaultAccelerometerGyroscopeAndMagnetometerMseRule.class,
                optimizer.getMseRule().getClass());
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
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
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelPsd(),
                0.0);
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelRootPsd(),
                0.0);
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
        assertNull(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedAccelerometerBiasFxVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFyVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFzVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiases());
        assertNull(optimizer.getEstimatedAccelerometerMa());
        assertNull(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedGyroscopeBiasXVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasYVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasZVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiases());
        assertNull(optimizer.getEstimatedGyroscopeMg());
        assertNull(optimizer.getEstimatedGyroscopeGg());
        assertNull(optimizer.getEstimatedMagnetometerHardIron());
        assertNull(optimizer.getEstimatedMagnetometerMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA,
                optimizer.getProgressDelta(), 0.0);
    }

    @Test
    public void testConstructor7() {
        final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                new KnownGravityNormAccelerometerCalibrator();
        final EasyGyroscopeCalibrator gyroscopeCalibrator =
                new EasyGyroscopeCalibrator();
        final KnownPositionAndInstantMagnetometerCalibrator magnetometerCalibrator =
                new KnownPositionAndInstantMagnetometerCalibrator();
        final BrentSingleOptimizer mseOptimizer = new BrentSingleOptimizer();

        BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                        accelerometerCalibrator, gyroscopeCalibrator, magnetometerCalibrator,
                        mseOptimizer);

        // check default values
        assertSame(mseOptimizer, optimizer.getMseOptimizer());
        assertSame(accelerometerCalibrator, optimizer.getAccelerometerCalibrator());
        assertSame(gyroscopeCalibrator, optimizer.getGyroscopeCalibrator());
        assertSame(magnetometerCalibrator, optimizer.getMagnetometerCalibrator());
        assertNotNull(optimizer.getAccelerometerQualityScoreMapper());
        assertEquals(DefaultAccelerometerQualityScoreMapper.class,
                optimizer.getAccelerometerQualityScoreMapper().getClass());
        assertNotNull(optimizer.getGyroscopeQualityScoreMapper());
        assertEquals(DefaultGyroscopeQualityScoreMapper.class,
                optimizer.getGyroscopeQualityScoreMapper().getClass());
        assertNotNull(optimizer.getMagnetometerQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class,
                optimizer.getMagnetometerQualityScoreMapper().getClass());
        assertNotNull(optimizer.getMseRule());
        assertEquals(DefaultAccelerometerGyroscopeAndMagnetometerMseRule.class,
                optimizer.getMseRule().getClass());
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
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
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelPsd(),
                0.0);
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelRootPsd(),
                0.0);
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
        assertNull(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedAccelerometerBiasFxVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFyVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFzVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiases());
        assertNull(optimizer.getEstimatedAccelerometerMa());
        assertNull(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedGyroscopeBiasXVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasYVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasZVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiases());
        assertNull(optimizer.getEstimatedGyroscopeMg());
        assertNull(optimizer.getEstimatedGyroscopeGg());
        assertNull(optimizer.getEstimatedMagnetometerHardIron());
        assertNull(optimizer.getEstimatedMagnetometerMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA,
                optimizer.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        optimizer = null;
        try {
            optimizer = new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                    new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(),
                    gyroscopeCalibrator, magnetometerCalibrator, mseOptimizer);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            optimizer = new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                    accelerometerCalibrator, new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(),
                    magnetometerCalibrator, mseOptimizer);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            optimizer = new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                    accelerometerCalibrator, gyroscopeCalibrator,
                    new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(),
                    mseOptimizer);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(optimizer);
    }

    @Test
    public void testConstructor8() {
        final AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
                mock(AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource.class);

        final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                new KnownGravityNormAccelerometerCalibrator();
        final EasyGyroscopeCalibrator gyroscopeCalibrator =
                new EasyGyroscopeCalibrator();
        final KnownPositionAndInstantMagnetometerCalibrator magnetometerCalibrator =
                new KnownPositionAndInstantMagnetometerCalibrator();
        final BrentSingleOptimizer mseOptimizer = new BrentSingleOptimizer();

        BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                        dataSource, accelerometerCalibrator, gyroscopeCalibrator,
                        magnetometerCalibrator, mseOptimizer);

        // check default values
        assertSame(mseOptimizer, optimizer.getMseOptimizer());
        assertSame(accelerometerCalibrator, optimizer.getAccelerometerCalibrator());
        assertSame(gyroscopeCalibrator, optimizer.getGyroscopeCalibrator());
        assertSame(magnetometerCalibrator, optimizer.getMagnetometerCalibrator());
        assertNotNull(optimizer.getAccelerometerQualityScoreMapper());
        assertEquals(DefaultAccelerometerQualityScoreMapper.class,
                optimizer.getAccelerometerQualityScoreMapper().getClass());
        assertNotNull(optimizer.getGyroscopeQualityScoreMapper());
        assertEquals(DefaultGyroscopeQualityScoreMapper.class,
                optimizer.getGyroscopeQualityScoreMapper().getClass());
        assertNotNull(optimizer.getMagnetometerQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class,
                optimizer.getMagnetometerQualityScoreMapper().getClass());
        assertNotNull(optimizer.getMseRule());
        assertEquals(DefaultAccelerometerGyroscopeAndMagnetometerMseRule.class,
                optimizer.getMseRule().getClass());
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
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
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelPsd(),
                0.0);
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelRootPsd(),
                0.0);
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
        assertNull(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedAccelerometerBiasFxVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFyVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFzVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiases());
        assertNull(optimizer.getEstimatedAccelerometerMa());
        assertNull(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedGyroscopeBiasXVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasYVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasZVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiases());
        assertNull(optimizer.getEstimatedGyroscopeMg());
        assertNull(optimizer.getEstimatedGyroscopeGg());
        assertNull(optimizer.getEstimatedMagnetometerHardIron());
        assertNull(optimizer.getEstimatedMagnetometerMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA,
                optimizer.getProgressDelta(), 0.0);

        // Force IllegalArgumentException
        optimizer = null;
        try {
            optimizer = new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                    dataSource, new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(),
                    gyroscopeCalibrator, magnetometerCalibrator, mseOptimizer);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            optimizer = new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                    dataSource, accelerometerCalibrator,
                    new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(),
                    magnetometerCalibrator, mseOptimizer);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            optimizer = new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                    dataSource, accelerometerCalibrator, gyroscopeCalibrator,
                    new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(),
                    mseOptimizer);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(optimizer);
    }

    @Test
    public void testGetSetMseOptimizer() throws LockedException {
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

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
    public void testGetSetAccelerometerCalibrator() throws LockedException {
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNull(optimizer.getAccelerometerCalibrator());

        // set new value
        final KnownGravityNormAccelerometerCalibrator calibrator =
                new KnownGravityNormAccelerometerCalibrator();

        optimizer.setAccelerometerCalibrator(calibrator);

        // check
        assertSame(calibrator, optimizer.getAccelerometerCalibrator());

        // Force IllegalArgumentException
        try {
            optimizer.setAccelerometerCalibrator(
                    new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetGyroscopeCalibrator() throws LockedException {
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNull(optimizer.getGyroscopeCalibrator());

        // set new value
        final EasyGyroscopeCalibrator calibrator = new EasyGyroscopeCalibrator();

        optimizer.setGyroscopeCalibrator(calibrator);

        // check
        assertSame(calibrator, optimizer.getGyroscopeCalibrator());

        // Force IllegalArgumentException
        try {
            optimizer.setGyroscopeCalibrator(
                    new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMagnetometerCalibrator() throws LockedException {
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNull(optimizer.getMagnetometerCalibrator());

        // set new value
        final KnownPositionAndInstantMagnetometerCalibrator calibrator =
                new KnownPositionAndInstantMagnetometerCalibrator();

        optimizer.setMagnetometerCalibrator(calibrator);

        // check
        assertSame(calibrator, optimizer.getMagnetometerCalibrator());

        // Force IllegalArgumentException
        try {
            optimizer.setMagnetometerCalibrator(
                    new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator());
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAccelerometerQualityScoreMapper() throws LockedException {
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNotNull(optimizer.getAccelerometerQualityScoreMapper());
        assertEquals(DefaultAccelerometerQualityScoreMapper.class,
                optimizer.getAccelerometerQualityScoreMapper().getClass());

        // set new value
        //noinspection unchecked
        final QualityScoreMapper<StandardDeviationBodyKinematics> qualityScoreMapper =
                mock(QualityScoreMapper.class);
        optimizer.setAccelerometerQualityScoreMapper(qualityScoreMapper);

        // check
        assertSame(qualityScoreMapper, optimizer.getAccelerometerQualityScoreMapper());
    }

    @Test
    public void testGetSetGyroscopeQualityScoreMapper() throws LockedException {
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNotNull(optimizer.getGyroscopeQualityScoreMapper());
        assertEquals(DefaultGyroscopeQualityScoreMapper.class,
                optimizer.getGyroscopeQualityScoreMapper().getClass());

        // set new value
        //noinspection unchecked
        final QualityScoreMapper<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> qualityScoreMapper =
                mock(QualityScoreMapper.class);
        optimizer.setGyroscopeQualityScoreMapper(qualityScoreMapper);

        // check
        assertSame(qualityScoreMapper, optimizer.getGyroscopeQualityScoreMapper());
    }

    @Test
    public void testGetSetMagnetometerQualityScoreMapper() throws LockedException {
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNotNull(optimizer.getMagnetometerQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class,
                optimizer.getMagnetometerQualityScoreMapper().getClass());

        // set new value
        //noinspection unchecked
        final QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity> qualityScoreMapper =
                mock(QualityScoreMapper.class);
        optimizer.setMagnetometerQualityScoreMapper(qualityScoreMapper);

        // check
        assertSame(qualityScoreMapper, optimizer.getMagnetometerQualityScoreMapper());
    }

    @Test
    public void testGetSetMseRule() throws LockedException {
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNotNull(optimizer.getMseRule());
        assertEquals(DefaultAccelerometerGyroscopeAndMagnetometerMseRule.class,
                optimizer.getMseRule().getClass());

        // set new value
        final AccelerometerGyroscopeAndMagnetometerMseRule rule =
                mock(AccelerometerGyroscopeAndMagnetometerMseRule.class);
        optimizer.setMseRule(rule);

        // check
        assertSame(rule, optimizer.getMseRule());
    }

    @Test
    public void testGetSetThresholdFactorRange() throws LockedException {
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default values
        assertEquals(BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer
                        .DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer
                        .DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);

        // set new value
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
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default values
        assertNull(optimizer.getDataSource());

        // set new value
        final AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
                mock(AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource.class);

        optimizer.setDataSource(dataSource);

        // check
        assertSame(dataSource, optimizer.getDataSource());
    }

    @Test
    public void testIsReady() throws LockedException {
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertFalse(optimizer.isReady());

        // set data source
        final AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
                mock(AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource.class);
        optimizer.setDataSource(dataSource);

        // check
        assertFalse(optimizer.isReady());

        // set accelerometer calibrator
        final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                new KnownGravityNormAccelerometerCalibrator();
        optimizer.setAccelerometerCalibrator(accelerometerCalibrator);

        // check
        assertFalse(optimizer.isReady());

        // set gyroscope calibrator
        final EasyGyroscopeCalibrator gyroscopeCalibrator = new EasyGyroscopeCalibrator();
        optimizer.setGyroscopeCalibrator(gyroscopeCalibrator);

        // check
        assertFalse(optimizer.isReady());

        // set magnetometer calibrator
        final KnownPositionAndInstantMagnetometerCalibrator magnetometerCalibrator =
                new KnownPositionAndInstantMagnetometerCalibrator();
        optimizer.setMagnetometerCalibrator(magnetometerCalibrator);

        // check
        assertTrue(optimizer.isReady());

        // unset accelerometer quality score mapper
        optimizer.setAccelerometerQualityScoreMapper(null);

        // check
        assertFalse(optimizer.isReady());

        // set quality score mapper
        optimizer.setAccelerometerQualityScoreMapper(
                new DefaultAccelerometerQualityScoreMapper());

        // check
        assertTrue(optimizer.isReady());

        // unset gyroscope quality score mapper
        optimizer.setGyroscopeQualityScoreMapper(null);

        // check
        assertFalse(optimizer.isReady());

        // set quality score mapper
        optimizer.setGyroscopeQualityScoreMapper(
                new DefaultGyroscopeQualityScoreMapper());

        // check
        assertTrue(optimizer.isReady());

        // unset magnetometer quality score mapper
        optimizer.setMagnetometerQualityScoreMapper(null);

        // check
        assertFalse(optimizer.isReady());
    }

    @Test
    public void testGetSetTimeInterval() throws LockedException {
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

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
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

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
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

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
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

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
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

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
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

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
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

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
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

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
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

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
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNull(optimizer.getListener());

        // set new value
        optimizer.setListener(this);

        // check
        assertSame(this, optimizer.getListener());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer();

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
    public void testOptimizeMaCommonAxisWithNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, IntervalDetectorThresholdFactorOptimizerException,
            InvalidRotationMatrixException, IOException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mTimedBodyKinematicsAndMagneticFluxDensities.clear();
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            mMagnetometerGeneratorMeasurements.clear();

            // generate measurements

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final NEDFrame nedFrame = generateFrame();
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);
            final Date timestamp = new Date(createTimestamp(randomizer));
            NEDPosition nedPosition = createPosition(randomizer);
            generateTimedBodyKinematicsAndMagneticFluxDensity(nedFrame, ecefFrame, false, ma,
                    accelNoiseRootPSD, gyroNoiseRootPSD, numSequences,
                    numMeasurements, hardIron, mm, timestamp, nedPosition,
                    MAGNETOMETER_NOISE_STD);

            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator =
                    new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator(mGeneratorListener);

            for (TimedBodyKinematicsAndMagneticFluxDensity timedBodyKinematics :
                    mTimedBodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            final ECEFGravity gravity = ECEFGravityEstimator
                    .estimateGravityAndReturnNew(ecefFrame);

            // configure calibrators and data source
            final Matrix initialBa = new Matrix(3, 1);
            final Matrix initialMa = new Matrix(3, 3);
            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(
                            gravity.getNorm(), true, initialBa,
                            initialMa);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator gyroscopeCalibrator = new EasyGyroscopeCalibrator();
            gyroscopeCalibrator.setCommonAxisUsed(true);
            gyroscopeCalibrator.setGDependentCrossBiasesEstimated(false);
            gyroscopeCalibrator.setInitialBias(initialBg);
            gyroscopeCalibrator.setInitialMg(initialMg);
            gyroscopeCalibrator.setInitialGg(initialGg);

            KnownPositionAndInstantMagnetometerCalibrator magnetometerCalibrator =
                    new KnownPositionAndInstantMagnetometerCalibrator();
            magnetometerCalibrator.setPosition(nedPosition);
            magnetometerCalibrator.setCommonAxisUsed(true);
            magnetometerCalibrator.setTime(timestamp);

            // create optimizer
            final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                    new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                            mDataSource, accelerometerCalibrator, gyroscopeCalibrator,
                            magnetometerCalibrator);
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
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelPsd() > 0.0);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getGyroscopeBaseNoiseLevelPsd()),
                    optimizer.getGyroscopeBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
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
            assertTrue(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFxVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFyVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFzVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedAccelerometerBiases());

            final Matrix optimalBa = Matrix.newFromArray(optimizer.getEstimatedAccelerometerBiases());
            final Matrix optimalMa = optimizer.getEstimatedAccelerometerMa();

            assertNotNull(optimalBa);
            assertNotNull(optimalMa);

            if (!ba.equals(optimalBa, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(optimalMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(optimalBa, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(optimalMa, ABSOLUTE_ERROR));

            assertTrue(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasXVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasYVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasZVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedGyroscopeBiases());

            final Matrix optimalBg = Matrix.newFromArray(optimizer.getEstimatedGyroscopeBiases());
            final Matrix optimalMg = optimizer.getEstimatedGyroscopeMg();
            final Matrix optimalGg = optimizer.getEstimatedGyroscopeGg();

            assertNotNull(optimalBg);
            assertNotNull(optimalMg);
            assertNotNull(optimalGg);

            if (!bg.equals(optimalBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(optimalMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(optimalGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(optimalBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(optimalMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(optimalGg, 0.0));

            final Matrix optimalHardIron = Matrix.newFromArray(optimizer.getEstimatedMagnetometerHardIron());
            final Matrix optimalMm = optimizer.getEstimatedMagnetometerMm();

            assertNotNull(optimalHardIron);
            assertNotNull(optimalMm);

            if (!hardIron.equals(optimalHardIron, LARGE_ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            if (!mm.equals(optimalMm, VERY_LARGE_ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            assertTrue(hardIron.equals(optimalHardIron, LARGE_ABSOLUTE_ERROR_MAGNETOMETER));
            assertTrue(mm.equals(optimalMm, VERY_LARGE_ABSOLUTE_ERROR_MAGNETOMETER));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            mMagnetometerGeneratorMeasurements.clear();
            generator.reset();
            generator.setThresholdFactor(thresholdFactor);

            for (TimedBodyKinematicsAndMagneticFluxDensity timedBodyKinematics :
                    mTimedBodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            accelerometerCalibrator.setMeasurements(mAccelerometerGeneratorMeasurements);
            gyroscopeCalibrator.setSequences(mGyroscopeGeneratorMeasurements);
            magnetometerCalibrator.setMeasurements(mMagnetometerGeneratorMeasurements);

            // calibrate
            try {
                accelerometerCalibrator.calibrate();
                gyroscopeCalibrator.calibrate();
                magnetometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            // check calibration resultç
            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final Matrix estimatedBg = gyroscopeCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = gyroscopeCalibrator.getEstimatedMg();
            final Matrix estimatedGg = gyroscopeCalibrator.getEstimatedGg();

            final Matrix estimatedHardIron = magnetometerCalibrator.getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = magnetometerCalibrator.getEstimatedMm();

            if (!ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, 0.0));

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
    public void testOptimizeGeneralNoGDependentCrossBiasesWithSmallNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, IntervalDetectorThresholdFactorOptimizerException,
            InvalidRotationMatrixException, IOException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double gyroNoiseRootPSD = 0.0;

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mTimedBodyKinematicsAndMagneticFluxDensities.clear();
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            mMagnetometerGeneratorMeasurements.clear();

            // generate measurements

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final NEDFrame nedFrame = generateFrame();
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronGeneral();
            assertNotNull(mm);
            final Date timestamp = new Date(createTimestamp(randomizer));
            NEDPosition nedPosition = createPosition(randomizer);
            generateTimedBodyKinematicsAndMagneticFluxDensity(nedFrame, ecefFrame, false, ma,
                    SMALL_ROOT_PSD, gyroNoiseRootPSD, numSequences,
                    numMeasurements, hardIron, mm, timestamp, nedPosition,
                    SMALL_MAGNETOMETER_NOISE_STD);

            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator =
                    new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator(mGeneratorListener);

            for (TimedBodyKinematicsAndMagneticFluxDensity timedBodyKinematics :
                    mTimedBodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            final ECEFGravity gravity = ECEFGravityEstimator
                    .estimateGravityAndReturnNew(ecefFrame);

            // configure calibrators and data source
            final Matrix initialBa = new Matrix(3, 1);
            final Matrix initialMa = new Matrix(3, 3);
            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(
                            gravity.getNorm(), false, initialBa,
                            initialMa);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator gyroscopeCalibrator = new EasyGyroscopeCalibrator();
            gyroscopeCalibrator.setCommonAxisUsed(true);
            gyroscopeCalibrator.setGDependentCrossBiasesEstimated(false);
            gyroscopeCalibrator.setInitialBias(initialBg);
            gyroscopeCalibrator.setInitialMg(initialMg);
            gyroscopeCalibrator.setInitialGg(initialGg);

            KnownPositionAndInstantMagnetometerCalibrator magnetometerCalibrator =
                    new KnownPositionAndInstantMagnetometerCalibrator();
            magnetometerCalibrator.setPosition(nedPosition);
            magnetometerCalibrator.setCommonAxisUsed(false);
            magnetometerCalibrator.setTime(timestamp);

            // create optimizer
            final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                    new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                            mDataSource, accelerometerCalibrator, gyroscopeCalibrator,
                            magnetometerCalibrator);
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
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelPsd() > 0.0);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getGyroscopeBaseNoiseLevelPsd()),
                    optimizer.getGyroscopeBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
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
            assertTrue(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFxVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFyVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFzVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedAccelerometerBiases());

            final Matrix optimalBa = Matrix.newFromArray(optimizer.getEstimatedAccelerometerBiases());
            final Matrix optimalMa = optimizer.getEstimatedAccelerometerMa();

            assertNotNull(optimalBa);
            assertNotNull(optimalMa);

            if (!ba.equals(optimalBa, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(optimalMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(optimalBa, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(optimalMa, ABSOLUTE_ERROR));

            assertTrue(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasXVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasYVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasZVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedGyroscopeBiases());

            final Matrix optimalBg = Matrix.newFromArray(optimizer.getEstimatedGyroscopeBiases());
            final Matrix optimalMg = optimizer.getEstimatedGyroscopeMg();
            final Matrix optimalGg = optimizer.getEstimatedGyroscopeGg();

            assertNotNull(optimalBg);
            assertNotNull(optimalMg);
            assertNotNull(optimalGg);

            if (!bg.equals(optimalBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(optimalMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(optimalGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(optimalBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(optimalMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(optimalGg, 0.0));

            final Matrix optimalHardIron = Matrix.newFromArray(optimizer.getEstimatedMagnetometerHardIron());
            final Matrix optimalMm = optimizer.getEstimatedMagnetometerMm();

            assertNotNull(optimalHardIron);
            assertNotNull(optimalMm);

            if (!hardIron.equals(optimalHardIron, LARGE_ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            if (!mm.equals(optimalMm, VERY_LARGE_ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            assertTrue(hardIron.equals(optimalHardIron, LARGE_ABSOLUTE_ERROR_MAGNETOMETER));
            assertTrue(mm.equals(optimalMm, VERY_LARGE_ABSOLUTE_ERROR_MAGNETOMETER));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            mMagnetometerGeneratorMeasurements.clear();
            generator.reset();
            generator.setThresholdFactor(thresholdFactor);

            for (TimedBodyKinematicsAndMagneticFluxDensity timedBodyKinematics :
                    mTimedBodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            accelerometerCalibrator.setMeasurements(mAccelerometerGeneratorMeasurements);
            gyroscopeCalibrator.setSequences(mGyroscopeGeneratorMeasurements);
            magnetometerCalibrator.setMeasurements(mMagnetometerGeneratorMeasurements);

            // calibrate
            try {
                accelerometerCalibrator.calibrate();
                gyroscopeCalibrator.calibrate();
                magnetometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            // check calibration resultç
            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final Matrix estimatedBg = gyroscopeCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = gyroscopeCalibrator.getEstimatedMg();
            final Matrix estimatedGg = gyroscopeCalibrator.getEstimatedGg();

            final Matrix estimatedHardIron = magnetometerCalibrator.getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = magnetometerCalibrator.getEstimatedMm();

            if (!ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, 0.0));

            if (!hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            if (!mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            assertTrue(hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR_MAGNETOMETER));
            assertTrue(mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR_MAGNETOMETER));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testOptimizeCommonAxisNoGDependentCrossBiasesWithSmallNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, IntervalDetectorThresholdFactorOptimizerException,
            InvalidRotationMatrixException, IOException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double gyroNoiseRootPSD = 0.0;

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mTimedBodyKinematicsAndMagneticFluxDensities.clear();
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            mMagnetometerGeneratorMeasurements.clear();

            // generate measurements

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final NEDFrame nedFrame = generateFrame();
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);
            final Date timestamp = new Date(createTimestamp(randomizer));
            NEDPosition nedPosition = createPosition(randomizer);
            generateTimedBodyKinematicsAndMagneticFluxDensity(nedFrame, ecefFrame, false, ma,
                    SMALL_ROOT_PSD, gyroNoiseRootPSD, numSequences,
                    numMeasurements, hardIron, mm, timestamp, nedPosition,
                    SMALL_MAGNETOMETER_NOISE_STD);

            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator =
                    new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator(mGeneratorListener);

            for (TimedBodyKinematicsAndMagneticFluxDensity timedBodyKinematics :
                    mTimedBodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            final ECEFGravity gravity = ECEFGravityEstimator
                    .estimateGravityAndReturnNew(ecefFrame);

            // configure calibrators and data source
            final Matrix initialBa = new Matrix(3, 1);
            final Matrix initialMa = new Matrix(3, 3);
            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(
                            gravity.getNorm(), true, initialBa,
                            initialMa);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator gyroscopeCalibrator = new EasyGyroscopeCalibrator();
            gyroscopeCalibrator.setCommonAxisUsed(true);
            gyroscopeCalibrator.setGDependentCrossBiasesEstimated(false);
            gyroscopeCalibrator.setInitialBias(initialBg);
            gyroscopeCalibrator.setInitialMg(initialMg);
            gyroscopeCalibrator.setInitialGg(initialGg);

            KnownPositionAndInstantMagnetometerCalibrator magnetometerCalibrator =
                    new KnownPositionAndInstantMagnetometerCalibrator();
            magnetometerCalibrator.setPosition(nedPosition);
            magnetometerCalibrator.setCommonAxisUsed(true);
            magnetometerCalibrator.setTime(timestamp);

            // create optimizer
            final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                    new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                            mDataSource, accelerometerCalibrator, gyroscopeCalibrator,
                            magnetometerCalibrator);
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
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelPsd() > 0.0);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getGyroscopeBaseNoiseLevelPsd()),
                    optimizer.getGyroscopeBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
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
            assertTrue(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFxVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFyVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFzVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedAccelerometerBiases());

            final Matrix optimalBa = Matrix.newFromArray(optimizer.getEstimatedAccelerometerBiases());
            final Matrix optimalMa = optimizer.getEstimatedAccelerometerMa();

            assertNotNull(optimalBa);
            assertNotNull(optimalMa);

            if (!ba.equals(optimalBa, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(optimalMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(optimalBa, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(optimalMa, ABSOLUTE_ERROR));

            assertTrue(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasXVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasYVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasZVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedGyroscopeBiases());

            final Matrix optimalBg = Matrix.newFromArray(optimizer.getEstimatedGyroscopeBiases());
            final Matrix optimalMg = optimizer.getEstimatedGyroscopeMg();
            final Matrix optimalGg = optimizer.getEstimatedGyroscopeGg();

            assertNotNull(optimalBg);
            assertNotNull(optimalMg);
            assertNotNull(optimalGg);

            if (!bg.equals(optimalBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(optimalMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(optimalGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(optimalBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(optimalMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(optimalGg, 0.0));

            final Matrix optimalHardIron = Matrix.newFromArray(optimizer.getEstimatedMagnetometerHardIron());
            final Matrix optimalMm = optimizer.getEstimatedMagnetometerMm();

            assertNotNull(optimalHardIron);
            assertNotNull(optimalMm);

            if (!hardIron.equals(optimalHardIron, ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            if (!mm.equals(optimalMm, LARGE_ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            assertTrue(hardIron.equals(optimalHardIron, ABSOLUTE_ERROR_MAGNETOMETER));
            assertTrue(mm.equals(optimalMm, LARGE_ABSOLUTE_ERROR_MAGNETOMETER));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            mMagnetometerGeneratorMeasurements.clear();
            generator.reset();
            generator.setThresholdFactor(thresholdFactor);

            for (TimedBodyKinematicsAndMagneticFluxDensity timedBodyKinematics :
                    mTimedBodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            accelerometerCalibrator.setMeasurements(mAccelerometerGeneratorMeasurements);
            gyroscopeCalibrator.setSequences(mGyroscopeGeneratorMeasurements);
            magnetometerCalibrator.setMeasurements(mMagnetometerGeneratorMeasurements);

            // calibrate
            try {
                accelerometerCalibrator.calibrate();
                gyroscopeCalibrator.calibrate();
                magnetometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            // check calibration resultç
            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final Matrix estimatedBg = gyroscopeCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = gyroscopeCalibrator.getEstimatedMg();
            final Matrix estimatedGg = gyroscopeCalibrator.getEstimatedGg();

            final Matrix estimatedHardIron = magnetometerCalibrator.getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = magnetometerCalibrator.getEstimatedMm();

            if (!ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, 0.0));

            if (!hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            if (!mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            assertTrue(hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR_MAGNETOMETER));
            assertTrue(mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR_MAGNETOMETER));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testOptimizeGeneralGDependentCrossBiasesWithSmallNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, IntervalDetectorThresholdFactorOptimizerException,
            InvalidRotationMatrixException, IOException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double gyroNoiseRootPSD = 0.0;

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mTimedBodyKinematicsAndMagneticFluxDensities.clear();
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            mMagnetometerGeneratorMeasurements.clear();

            // generate measurements

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final NEDFrame nedFrame = generateFrame();
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS_AND_CROSS_BIASES;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronGeneral();
            assertNotNull(mm);
            final Date timestamp = new Date(createTimestamp(randomizer));
            NEDPosition nedPosition = createPosition(randomizer);
            generateTimedBodyKinematicsAndMagneticFluxDensity(nedFrame, ecefFrame, false, ma,
                    SMALL_ROOT_PSD, gyroNoiseRootPSD, numSequences,
                    numMeasurements, hardIron, mm, timestamp, nedPosition,
                    SMALL_MAGNETOMETER_NOISE_STD);

            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator =
                    new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator(mGeneratorListener);

            for (TimedBodyKinematicsAndMagneticFluxDensity timedBodyKinematics :
                    mTimedBodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            final ECEFGravity gravity = ECEFGravityEstimator
                    .estimateGravityAndReturnNew(ecefFrame);

            // configure calibrators and data source
            final Matrix initialBa = new Matrix(3, 1);
            final Matrix initialMa = new Matrix(3, 3);
            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(
                            gravity.getNorm(), false, initialBa,
                            initialMa);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator gyroscopeCalibrator = new EasyGyroscopeCalibrator();
            gyroscopeCalibrator.setCommonAxisUsed(true);
            gyroscopeCalibrator.setGDependentCrossBiasesEstimated(true);
            gyroscopeCalibrator.setInitialBias(initialBg);
            gyroscopeCalibrator.setInitialMg(initialMg);
            gyroscopeCalibrator.setInitialGg(initialGg);

            KnownPositionAndInstantMagnetometerCalibrator magnetometerCalibrator =
                    new KnownPositionAndInstantMagnetometerCalibrator();
            magnetometerCalibrator.setPosition(nedPosition);
            magnetometerCalibrator.setCommonAxisUsed(false);
            magnetometerCalibrator.setTime(timestamp);

            // create optimizer
            final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                    new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                            mDataSource, accelerometerCalibrator, gyroscopeCalibrator,
                            magnetometerCalibrator);
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
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelPsd() > 0.0);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getGyroscopeBaseNoiseLevelPsd()),
                    optimizer.getGyroscopeBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
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
            assertTrue(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFxVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFyVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFzVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedAccelerometerBiases());

            final Matrix optimalBa = Matrix.newFromArray(optimizer.getEstimatedAccelerometerBiases());
            final Matrix optimalMa = optimizer.getEstimatedAccelerometerMa();

            assertNotNull(optimalBa);
            assertNotNull(optimalMa);

            if (!ba.equals(optimalBa, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(optimalMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(optimalBa, SMALL_ABSOLUTE_ERROR));
            assertTrue(ma.equals(optimalMa, ABSOLUTE_ERROR));

            assertTrue(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasXVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasYVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasZVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedGyroscopeBiases());

            final Matrix optimalBg = Matrix.newFromArray(optimizer.getEstimatedGyroscopeBiases());
            final Matrix optimalMg = optimizer.getEstimatedGyroscopeMg();
            final Matrix optimalGg = optimizer.getEstimatedGyroscopeGg();

            assertNotNull(optimalBg);
            assertNotNull(optimalMg);
            assertNotNull(optimalGg);

            if (!bg.equals(optimalBg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(optimalMg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(optimalGg, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(optimalBg, ABSOLUTE_ERROR));
            assertTrue(mg.equals(optimalMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(optimalGg, SMALL_ABSOLUTE_ERROR));

            final Matrix optimalHardIron = Matrix.newFromArray(optimizer.getEstimatedMagnetometerHardIron());
            final Matrix optimalMm = optimizer.getEstimatedMagnetometerMm();

            assertNotNull(optimalHardIron);
            assertNotNull(optimalMm);

            if (!hardIron.equals(optimalHardIron, LARGE_ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            if (!mm.equals(optimalMm, VERY_LARGE_ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            assertTrue(hardIron.equals(optimalHardIron, LARGE_ABSOLUTE_ERROR_MAGNETOMETER));
            assertTrue(mm.equals(optimalMm, VERY_LARGE_ABSOLUTE_ERROR_MAGNETOMETER));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            mMagnetometerGeneratorMeasurements.clear();
            generator.reset();
            generator.setThresholdFactor(thresholdFactor);

            for (TimedBodyKinematicsAndMagneticFluxDensity timedBodyKinematics :
                    mTimedBodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            accelerometerCalibrator.setMeasurements(mAccelerometerGeneratorMeasurements);
            gyroscopeCalibrator.setSequences(mGyroscopeGeneratorMeasurements);
            magnetometerCalibrator.setMeasurements(mMagnetometerGeneratorMeasurements);

            // calibrate
            try {
                accelerometerCalibrator.calibrate();
                gyroscopeCalibrator.calibrate();
                magnetometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            // check calibration resultç
            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final Matrix estimatedBg = gyroscopeCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = gyroscopeCalibrator.getEstimatedMg();
            final Matrix estimatedGg = gyroscopeCalibrator.getEstimatedGg();

            final Matrix estimatedHardIron = magnetometerCalibrator.getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = magnetometerCalibrator.getEstimatedMm();

            if (!ba.equals(estimatedBa, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, SMALL_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            if (!bg.equals(estimatedBg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, SMALL_ABSOLUTE_ERROR));

            if (!hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            assertTrue(hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR_MAGNETOMETER));
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR_MAGNETOMETER));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testOptimizeCommonAxisGDependentCrossBiasesWithSmallNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, IntervalDetectorThresholdFactorOptimizerException,
            InvalidRotationMatrixException, IOException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double gyroNoiseRootPSD = 0.0;

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mTimedBodyKinematicsAndMagneticFluxDensities.clear();
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            mMagnetometerGeneratorMeasurements.clear();

            // generate measurements

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final NEDFrame nedFrame = generateFrame();
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS_AND_CROSS_BIASES;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);
            final Date timestamp = new Date(createTimestamp(randomizer));
            NEDPosition nedPosition = createPosition(randomizer);
            generateTimedBodyKinematicsAndMagneticFluxDensity(nedFrame, ecefFrame, false, ma,
                    SMALL_ROOT_PSD, gyroNoiseRootPSD, numSequences,
                    numMeasurements, hardIron, mm, timestamp, nedPosition,
                    SMALL_MAGNETOMETER_NOISE_STD);

            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator =
                    new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator(mGeneratorListener);

            for (TimedBodyKinematicsAndMagneticFluxDensity timedBodyKinematics :
                    mTimedBodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            final ECEFGravity gravity = ECEFGravityEstimator
                    .estimateGravityAndReturnNew(ecefFrame);

            // configure calibrators and data source
            final Matrix initialBa = new Matrix(3, 1);
            final Matrix initialMa = new Matrix(3, 3);
            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(
                            gravity.getNorm(), true, initialBa,
                            initialMa);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator gyroscopeCalibrator = new EasyGyroscopeCalibrator();
            gyroscopeCalibrator.setCommonAxisUsed(true);
            gyroscopeCalibrator.setGDependentCrossBiasesEstimated(true);
            gyroscopeCalibrator.setInitialBias(initialBg);
            gyroscopeCalibrator.setInitialMg(initialMg);
            gyroscopeCalibrator.setInitialGg(initialGg);

            KnownPositionAndInstantMagnetometerCalibrator magnetometerCalibrator =
                    new KnownPositionAndInstantMagnetometerCalibrator();
            magnetometerCalibrator.setPosition(nedPosition);
            magnetometerCalibrator.setCommonAxisUsed(true);
            magnetometerCalibrator.setTime(timestamp);

            // create optimizer
            final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                    new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                            mDataSource, accelerometerCalibrator, gyroscopeCalibrator,
                            magnetometerCalibrator);
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
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelPsd() > 0.0);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getGyroscopeBaseNoiseLevelPsd()),
                    optimizer.getGyroscopeBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
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
            assertTrue(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFxVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFyVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFzVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedAccelerometerBiases());

            final Matrix optimalBa = Matrix.newFromArray(optimizer.getEstimatedAccelerometerBiases());
            final Matrix optimalMa = optimizer.getEstimatedAccelerometerMa();

            assertNotNull(optimalBa);
            assertNotNull(optimalMa);

            if (!ba.equals(optimalBa, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(optimalMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(optimalBa, SMALL_ABSOLUTE_ERROR));
            assertTrue(ma.equals(optimalMa, ABSOLUTE_ERROR));

            assertTrue(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasXVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasYVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasZVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedGyroscopeBiases());

            final Matrix optimalBg = Matrix.newFromArray(optimizer.getEstimatedGyroscopeBiases());
            final Matrix optimalMg = optimizer.getEstimatedGyroscopeMg();
            final Matrix optimalGg = optimizer.getEstimatedGyroscopeGg();

            assertNotNull(optimalBg);
            assertNotNull(optimalMg);
            assertNotNull(optimalGg);

            if (!bg.equals(optimalBg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(optimalMg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(optimalGg, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(optimalBg, ABSOLUTE_ERROR));
            assertTrue(mg.equals(optimalMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(optimalGg, SMALL_ABSOLUTE_ERROR));

            final Matrix optimalHardIron = Matrix.newFromArray(optimizer.getEstimatedMagnetometerHardIron());
            final Matrix optimalMm = optimizer.getEstimatedMagnetometerMm();

            assertNotNull(optimalHardIron);
            assertNotNull(optimalMm);

            if (!hardIron.equals(optimalHardIron, LARGE_ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            if (!mm.equals(optimalMm, VERY_LARGE_ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            assertTrue(hardIron.equals(optimalHardIron, LARGE_ABSOLUTE_ERROR_MAGNETOMETER));
            assertTrue(mm.equals(optimalMm, VERY_LARGE_ABSOLUTE_ERROR_MAGNETOMETER));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            mMagnetometerGeneratorMeasurements.clear();
            generator.reset();
            generator.setThresholdFactor(thresholdFactor);

            for (TimedBodyKinematicsAndMagneticFluxDensity timedBodyKinematics :
                    mTimedBodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            accelerometerCalibrator.setMeasurements(mAccelerometerGeneratorMeasurements);
            gyroscopeCalibrator.setSequences(mGyroscopeGeneratorMeasurements);
            magnetometerCalibrator.setMeasurements(mMagnetometerGeneratorMeasurements);

            // calibrate
            try {
                accelerometerCalibrator.calibrate();
                gyroscopeCalibrator.calibrate();
                magnetometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            // check calibration resultç
            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final Matrix estimatedBg = gyroscopeCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = gyroscopeCalibrator.getEstimatedMg();
            final Matrix estimatedGg = gyroscopeCalibrator.getEstimatedGg();

            final Matrix estimatedHardIron = magnetometerCalibrator.getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = magnetometerCalibrator.getEstimatedMm();

            if (!ba.equals(estimatedBa, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, SMALL_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            if (!bg.equals(estimatedBg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, SMALL_ABSOLUTE_ERROR));

            if (!hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            assertTrue(hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR_MAGNETOMETER));
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR_MAGNETOMETER));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testOptimizeCommonAxisNoGDependentCrossBiasesWithSmallNoiseRotationAndPositionChange()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException,
            IntervalDetectorThresholdFactorOptimizerException,
            InvalidRotationMatrixException, IOException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double gyroNoiseRootPSD = 0.0;

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mTimedBodyKinematicsAndMagneticFluxDensities.clear();
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            mMagnetometerGeneratorMeasurements.clear();

            // generate measurements

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final NEDFrame nedFrame = generateFrame();
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);
            final Date timestamp = new Date(createTimestamp(randomizer));
            NEDPosition nedPosition = createPosition(randomizer);
            generateTimedBodyKinematicsAndMagneticFluxDensity(nedFrame, ecefFrame, true, ma,
                    SMALL_ROOT_PSD, gyroNoiseRootPSD, numSequences,
                    numMeasurements, hardIron, mm, timestamp, nedPosition,
                    SMALL_MAGNETOMETER_NOISE_STD);

            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator =
                    new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator(mGeneratorListener);

            for (TimedBodyKinematicsAndMagneticFluxDensity timedBodyKinematics :
                    mTimedBodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            final ECEFGravity gravity = ECEFGravityEstimator
                    .estimateGravityAndReturnNew(ecefFrame);

            // configure calibrators and data source
            final Matrix initialBa = new Matrix(3, 1);
            final Matrix initialMa = new Matrix(3, 3);
            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(
                            gravity.getNorm(), true, initialBa,
                            initialMa);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator gyroscopeCalibrator = new EasyGyroscopeCalibrator();
            gyroscopeCalibrator.setCommonAxisUsed(true);
            gyroscopeCalibrator.setGDependentCrossBiasesEstimated(false);
            gyroscopeCalibrator.setInitialBias(initialBg);
            gyroscopeCalibrator.setInitialMg(initialMg);
            gyroscopeCalibrator.setInitialGg(initialGg);

            KnownPositionAndInstantMagnetometerCalibrator magnetometerCalibrator =
                    new KnownPositionAndInstantMagnetometerCalibrator();
            magnetometerCalibrator.setPosition(nedPosition);
            magnetometerCalibrator.setCommonAxisUsed(true);
            magnetometerCalibrator.setTime(timestamp);

            // create optimizer
            final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                    new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                            mDataSource, accelerometerCalibrator, gyroscopeCalibrator,
                            magnetometerCalibrator);
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
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelPsd() > 0.0);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getGyroscopeBaseNoiseLevelPsd()),
                    optimizer.getGyroscopeBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
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
            assertTrue(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFxVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFyVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFzVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedAccelerometerBiases());

            final Matrix optimalBa = Matrix.newFromArray(optimizer.getEstimatedAccelerometerBiases());
            final Matrix optimalMa = optimizer.getEstimatedAccelerometerMa();

            assertNotNull(optimalBa);
            assertNotNull(optimalMa);

            if (!ba.equals(optimalBa, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(optimalMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(optimalBa, ABSOLUTE_ERROR));
            assertTrue(ma.equals(optimalMa, ABSOLUTE_ERROR));

            assertTrue(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasXVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasYVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasZVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedGyroscopeBiases());

            final Matrix optimalBg = Matrix.newFromArray(optimizer.getEstimatedGyroscopeBiases());
            final Matrix optimalMg = optimizer.getEstimatedGyroscopeMg();
            final Matrix optimalGg = optimizer.getEstimatedGyroscopeGg();

            assertNotNull(optimalBg);
            assertNotNull(optimalMg);
            assertNotNull(optimalGg);

            if (!bg.equals(optimalBg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(optimalMg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(optimalGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(optimalBg, ABSOLUTE_ERROR));
            assertTrue(mg.equals(optimalMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(optimalGg, 0.0));

            final Matrix optimalHardIron = Matrix.newFromArray(optimizer.getEstimatedMagnetometerHardIron());
            final Matrix optimalMm = optimizer.getEstimatedMagnetometerMm();

            assertNotNull(optimalHardIron);
            assertNotNull(optimalMm);

            if (!hardIron.equals(optimalHardIron, ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            if (!mm.equals(optimalMm, LARGE_ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            assertTrue(hardIron.equals(optimalHardIron, ABSOLUTE_ERROR_MAGNETOMETER));
            assertTrue(mm.equals(optimalMm, LARGE_ABSOLUTE_ERROR_MAGNETOMETER));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            mMagnetometerGeneratorMeasurements.clear();
            generator.reset();
            generator.setThresholdFactor(thresholdFactor);

            for (TimedBodyKinematicsAndMagneticFluxDensity timedBodyKinematics :
                    mTimedBodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            accelerometerCalibrator.setMeasurements(mAccelerometerGeneratorMeasurements);
            gyroscopeCalibrator.setSequences(mGyroscopeGeneratorMeasurements);
            magnetometerCalibrator.setMeasurements(mMagnetometerGeneratorMeasurements);

            // calibrate
            try {
                accelerometerCalibrator.calibrate();
                gyroscopeCalibrator.calibrate();
                magnetometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            // check calibration resultç
            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final Matrix estimatedBg = gyroscopeCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = gyroscopeCalibrator.getEstimatedMg();
            final Matrix estimatedGg = gyroscopeCalibrator.getEstimatedGg();

            final Matrix estimatedHardIron = magnetometerCalibrator.getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = magnetometerCalibrator.getEstimatedMm();

            if (!ba.equals(estimatedBa, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            if (!bg.equals(estimatedBg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, 0.0));

            if (!hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            if (!mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            assertTrue(hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR_MAGNETOMETER));
            assertTrue(mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR_MAGNETOMETER));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testOptimizeRobustCalibrators() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, IntervalDetectorThresholdFactorOptimizerException,
            InvalidRotationMatrixException, IOException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double gyroNoiseRootPSD = 0.0;

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mTimedBodyKinematicsAndMagneticFluxDensities.clear();
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            mMagnetometerGeneratorMeasurements.clear();

            // generate measurements

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);

            final NEDFrame nedFrame = generateFrame();
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            final int numSequences = 3 * EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final int numMeasurements = 3 * KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMON_Z_AXIS;
            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);
            final Date timestamp = new Date(createTimestamp(randomizer));
            NEDPosition nedPosition = createPosition(randomizer);
            generateTimedBodyKinematicsAndMagneticFluxDensity(nedFrame, ecefFrame, false, ma,
                    SMALL_ROOT_PSD, gyroNoiseRootPSD, numSequences,
                    numMeasurements, hardIron, mm, timestamp, nedPosition,
                    SMALL_MAGNETOMETER_NOISE_STD);

            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator =
                    new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator(mGeneratorListener);

            for (TimedBodyKinematicsAndMagneticFluxDensity timedBodyKinematics :
                    mTimedBodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            final ECEFGravity gravity = ECEFGravityEstimator
                    .estimateGravityAndReturnNew(ecefFrame);

            // configure calibrators and data source
            final Matrix initialBa = new Matrix(3, 1);
            final Matrix initialMa = new Matrix(3, 3);
            final PROMedSRobustKnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new PROMedSRobustKnownGravityNormAccelerometerCalibrator();
            accelerometerCalibrator.setGroundTruthGravityNorm(gravity.getNorm());
            accelerometerCalibrator.setCommonAxisUsed(true);
            accelerometerCalibrator.setInitialBias(initialBa);
            accelerometerCalibrator.setInitialMa(initialMa);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final PROMedSRobustEasyGyroscopeCalibrator gyroscopeCalibrator =
                    new PROMedSRobustEasyGyroscopeCalibrator();
            gyroscopeCalibrator.setCommonAxisUsed(true);
            gyroscopeCalibrator.setGDependentCrossBiasesEstimated(false);
            gyroscopeCalibrator.setInitialBias(initialBg);
            gyroscopeCalibrator.setInitialMg(initialMg);
            gyroscopeCalibrator.setInitialGg(initialGg);

            PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator magnetometerCalibrator =
                    new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator();
            magnetometerCalibrator.setPosition(nedPosition);
            magnetometerCalibrator.setCommonAxisUsed(true);
            magnetometerCalibrator.setTime(timestamp);

            // create optimizer
            final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer =
                    new BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
                            mDataSource, accelerometerCalibrator, gyroscopeCalibrator,
                            magnetometerCalibrator);
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
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelPsd() > 0.0);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getGyroscopeBaseNoiseLevelPsd()),
                    optimizer.getGyroscopeBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
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
            assertTrue(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFxVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFyVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFzVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedAccelerometerBiases());

            final Matrix optimalBa = Matrix.newFromArray(optimizer.getEstimatedAccelerometerBiases());
            final Matrix optimalMa = optimizer.getEstimatedAccelerometerMa();

            assertNotNull(optimalBa);
            assertNotNull(optimalMa);

            if (!ba.equals(optimalBa, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(optimalMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(optimalBa, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(optimalMa, ABSOLUTE_ERROR));

            assertTrue(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasXVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasYVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasZVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedGyroscopeBiases());

            final Matrix optimalBg = Matrix.newFromArray(optimizer.getEstimatedGyroscopeBiases());
            final Matrix optimalMg = optimizer.getEstimatedGyroscopeMg();
            final Matrix optimalGg = optimizer.getEstimatedGyroscopeGg();

            assertNotNull(optimalBg);
            assertNotNull(optimalMg);
            assertNotNull(optimalGg);

            if (!bg.equals(optimalBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(optimalMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(optimalGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(optimalBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(optimalMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(optimalGg, 0.0));

            final Matrix optimalHardIron = Matrix.newFromArray(optimizer.getEstimatedMagnetometerHardIron());
            final Matrix optimalMm = optimizer.getEstimatedMagnetometerMm();

            assertNotNull(optimalHardIron);
            assertNotNull(optimalMm);

            if (!hardIron.equals(optimalHardIron, ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            if (!mm.equals(optimalMm, LARGE_ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            assertTrue(hardIron.equals(optimalHardIron, ABSOLUTE_ERROR_MAGNETOMETER));
            assertTrue(mm.equals(optimalMm, LARGE_ABSOLUTE_ERROR_MAGNETOMETER));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            mMagnetometerGeneratorMeasurements.clear();
            generator.reset();
            generator.setThresholdFactor(thresholdFactor);

            for (TimedBodyKinematicsAndMagneticFluxDensity timedBodyKinematics :
                    mTimedBodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            accelerometerCalibrator.setMeasurements(mAccelerometerGeneratorMeasurements);
            gyroscopeCalibrator.setSequences(mGyroscopeGeneratorMeasurements);
            magnetometerCalibrator.setMeasurements(mMagnetometerGeneratorMeasurements);

            // calibrate
            try {
                accelerometerCalibrator.calibrate();
                gyroscopeCalibrator.calibrate();
                magnetometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            // check calibration resultç
            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final Matrix estimatedBg = gyroscopeCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = gyroscopeCalibrator.getEstimatedMg();
            final Matrix estimatedGg = gyroscopeCalibrator.getEstimatedGg();

            final Matrix estimatedHardIron = magnetometerCalibrator.getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = magnetometerCalibrator.getEstimatedMm();

            if (!ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, 0.0));

            if (!hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            if (!mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR_MAGNETOMETER)) {
                continue;
            }
            assertTrue(hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR_MAGNETOMETER));
            assertTrue(mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR_MAGNETOMETER));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onOptimizeStart(
            final IntervalDetectorThresholdFactorOptimizer<TimedBodyKinematicsAndMagneticFluxDensity,
                    AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource> optimizer) {
        mStart++;
        checkLocked((BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer) optimizer);
    }

    @Override
    public void onOptimizeEnd(
            final IntervalDetectorThresholdFactorOptimizer<TimedBodyKinematicsAndMagneticFluxDensity,
                    AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource> optimizer) {
        mEnd++;
        checkLocked((BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer) optimizer);
    }

    @Override
    public void onOptimizeProgressChange(
            final IntervalDetectorThresholdFactorOptimizer<TimedBodyKinematicsAndMagneticFluxDensity,
                    AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource> optimizer,
            final float progress) {
        assertTrue(progress >= 0.0f);
        assertTrue(progress <= 1.0f);
        assertTrue(progress > mProgress);
        if (mProgress == 0.0f) {
            checkLocked((BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer) optimizer);
        }
        mProgress = progress;
    }

    private void checkLocked(
            final BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer) {
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
            optimizer.setAccelerometerCalibrator(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            optimizer.setGyroscopeCalibrator(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            optimizer.setAccelerometerQualityScoreMapper(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            optimizer.setGyroscopeQualityScoreMapper(null);
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

    @SuppressWarnings("SameParameterValue")
    private void generateTimedBodyKinematicsAndMagneticFluxDensity(
            final NEDFrame nedFrame, final ECEFFrame ecefFrame,
            final boolean changePosition, final Matrix ma, final double accelNoiseRootPSD,
            final double gyroNoiseRootPSD, final int numSequences,
            final int numMeasurements,
            final Matrix hardIron,
            final Matrix mm,
            final Date timestamp,
            final NEDPosition nedPosition,
            final double magnetometerNoiseStd) throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, IOException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
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

        CoordinateTransformation nedC = nedFrame.getCoordinateTransformation();
        CoordinateTransformation cnb = nedC.inverseAndReturnNew();

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                        ecefFrame, ecefFrame);

        // generate initial static samples
        final int initialStaticSamples = TriadStaticIntervalDetector
                .DEFAULT_INITIAL_STATIC_SAMPLES;
        generateStaticSamples(initialStaticSamples, trueKinematics, errors, random,
                0, hardIron, mm, wmmEstimator, timestamp, nedPosition,
                cnb, noiseRandomizer);

        final int n = Math.max(numSequences + 1, numMeasurements);

        final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
        final int dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

        int start = initialStaticSamples;
        for (int i = 0; i < n; i++) {
            nedFrame.getPosition(nedPosition);
            nedC = nedFrame.getCoordinateTransformation();
            cnb = nedC.inverseAndReturnNew();

            // generate static samples
            generateStaticSamples(staticPeriodLength, trueKinematics, errors, random,
                    start, hardIron, mm, wmmEstimator, timestamp, nedPosition,
                    cnb, noiseRandomizer);
            start += staticPeriodLength;

            // generate dynamic samples
            generateDynamicSamples(dynamicPeriodLength, trueKinematics,
                    randomizer, ecefFrame, nedFrame, errors, random, start,
                    changePosition, hardIron, mm, wmmEstimator, timestamp, nedPosition,
                    cnb, noiseRandomizer);
            start += dynamicPeriodLength;
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
                MIN_HEIGHT, MAX_HEIGHT);

        return new NEDPosition(latitude, longitude, height);
    }

    private static long createTimestamp(final UniformRandomizer randomizer) {
        return randomizer.nextLong(
                START_TIMESTAMP_MILLIS, END_TIMESTAMP_MILLIS);
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
            final int numSamples,
            final BodyKinematics trueKinematics,
            final IMUErrors errors,
            final Random random,
            final int startSample,
            final Matrix hardIron,
            final Matrix mm,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final Date timestamp,
            final NEDPosition nedPosition,
            final CoordinateTransformation cnb,
            final GaussianRandomizer noiseRandomizer) {

        for (int i = 0, j = startSample; i < numSamples; i++, j++) {

            final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                    .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                            random);

            final BodyMagneticFluxDensity b = generateB(hardIron.getBuffer(),
                    mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition,
                    cnb);

            final TimedBodyKinematicsAndMagneticFluxDensity tkb = new TimedBodyKinematicsAndMagneticFluxDensity();
            tkb.setKinematics(measuredKinematics);
            tkb.setMagneticFluxDensity(b);
            tkb.setTimestampSeconds(j * TIME_INTERVAL_SECONDS);

            mTimedBodyKinematicsAndMagneticFluxDensities.add(tkb);
        }
    }

    @SuppressWarnings("SameParameterValue")
    private void generateDynamicSamples(
            final int numSamples,
            final BodyKinematics trueKinematics,
            final UniformRandomizer randomizer,
            final ECEFFrame ecefFrame,
            final NEDFrame nedFrame,
            final IMUErrors errors,
            final Random random,
            final int startSample,
            final boolean changePosition,
            final Matrix hardIron,
            final Matrix mm,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final Date timestamp,
            final NEDPosition nedPosition,
            final CoordinateTransformation cnb,
            final GaussianRandomizer noiseRandomizer)
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
        final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
        final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

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

        final Quaternion beforeQ = new Quaternion();
        nedC.asRotation(beforeQ);

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

        final BodyKinematics measuredBeforeGravityKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS,
                        trueKinematics, errors, random);
        final double beforeMeanFx = measuredBeforeGravityKinematics.getFx();
        final double beforeMeanFy = measuredBeforeGravityKinematics.getFy();
        final double beforeMeanFz = measuredBeforeGravityKinematics.getFz();

        final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence<>();
        sequence.setBeforeMeanSpecificForceCoordinates(
                beforeMeanFx, beforeMeanFy, beforeMeanFz);

        final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                new BodyKinematicsSequence<>();
        final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList =
                new ArrayList<>();

        for (int i = 0, j = startSample; i < numSamples; i++, j++) {
            final double progress = (double) i / (double) numSamples;

            final double newRoll = oldRoll + interpolate(deltaRoll, progress);
            final double newPitch = oldPitch + interpolate(deltaPitch, progress);
            final double newYaw = oldYaw + interpolate(deltaYaw, progress);
            final CoordinateTransformation newNedC =
                    new CoordinateTransformation(
                            newRoll, newPitch, newYaw,
                            FrameType.BODY_FRAME,
                            FrameType.LOCAL_NAVIGATION_FRAME);
            final NEDPosition newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final double newEcefX = oldEcefX + interpolate(deltaX, progress);
            final double newEcefY = oldEcefY + interpolate(deltaY, progress);
            final double newEcefZ = oldEcefZ + interpolate(deltaZ, progress);

            newEcefFrame.setCoordinates(newEcefX, newEcefY, newEcefZ);

            ECEFtoNEDFrameConverter.convertECEFtoNED(newEcefFrame, newNedFrame);

            final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

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

            final TimedBodyKinematicsAndMagneticFluxDensity tkb = new TimedBodyKinematicsAndMagneticFluxDensity();
            tkb.setKinematics(measuredKinematics);
            tkb.setMagneticFluxDensity(b);
            tkb.setTimestampSeconds(timestampSeconds);

            mTimedBodyKinematicsAndMagneticFluxDensities.add(tkb);

            final StandardDeviationTimedBodyKinematics trueTimedKinematics =
                    new StandardDeviationTimedBodyKinematics(
                            new BodyKinematics(trueKinematics), timestampSeconds,
                            specificForceStandardDeviation,
                            angularRateStandardDeviation);
            trueTimedKinematicsList.add(trueTimedKinematics);

            oldNedFrame.copyFrom(newNedFrame);
            oldEcefFrame.copyFrom(newEcefFrame);
            oldRoll = newRoll;
            oldPitch = newPitch;
            oldYaw = newYaw;
            oldEcefX = newEcefX;
            oldEcefY = newEcefY;
            oldEcefZ = newEcefZ;
        }

        trueSequence.setItems(trueTimedKinematicsList);

        final Quaternion afterQ = new Quaternion();
        QuaternionIntegrator.integrateGyroSequence(
                trueSequence, beforeQ, afterQ);

        final CoordinateTransformation newNedC =
                new CoordinateTransformation(
                        afterQ.asInhomogeneousMatrix(),
                        FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);
        newNedFrame.setCoordinateTransformation(newNedC);

        NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);


        // update current ECEF and NED frames
        ecefFrame.copyFrom(newEcefFrame);
        nedFrame.copyFrom(newNedFrame);

        // after dynamic sequence finishes, update true kinematics for a
        // static sequence at current frame
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS,
                newEcefFrame, newEcefFrame, trueKinematics);
    }

    // This is required to simulate a smooth transition of values during
    // dynamic period, to avoid a sudden rotation or translation and simulate
    // a more natural behaviour.
    private double interpolate(final double value, final double progress) {
        return -2.0 * (Math.abs(progress - 0.5) - 0.5) * value;
    }

    private void reset() {
        mStart = 0;
        mEnd = 0;
        mProgress = 0.0f;
    }
}
