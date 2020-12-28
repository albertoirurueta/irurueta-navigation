package com.irurueta.navigation.inertial.calibration.generators;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.navigation.LockedException;
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
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.TimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownGravityNormAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.EasyGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionIntegrator;
import com.irurueta.navigation.inertial.calibration.intervals.AccelerationTriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;
import static org.junit.Assert.assertEquals;

public class AccelerometerAndGyroscopeMeasurementsGeneratorTest implements
        AccelerometerAndGyroscopeMeasurementsGeneratorListener {

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

    private static final double ABSOLUTE_ERROR = 5e-4;

    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;

    private static final double LARGE_ABSOLUTE_ERROR = 5e-3;

    private static final double SMALL_ABSOLUTE_ERROR = 1e-6;

    private static final double SMALL_ROOT_PSD = 1e-15;

    private int mInitializationStarted;
    private int mInitializationCompleted;
    private int mError;
    private int mStaticIntervalDetected;
    private int mDynamicIntervalDetected;
    private int mStaticIntervalSkipped;
    private int mDynamicIntervalSkipped;
    private int mGeneratedAccelerometerMeasurement;
    private int mGeneratedGyroscopeMeasurement;
    private int mReset;

    private final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> mGyroscopeMeasurements =
            new ArrayList<>();

    private final List<StandardDeviationBodyKinematics> mAccelerometerMeasurements =
            new ArrayList<>();

    @Test
    public void testConstructor1() {
        final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                new AccelerometerAndGyroscopeMeasurementsGenerator();

        // check default values
        assertEquals(generator.getTimeInterval(), TIME_INTERVAL_SECONDS,
                0.0);
        final Time timeInterval1 = generator.getTimeIntervalAsTime();
        assertEquals(timeInterval1.getValue().doubleValue(),
                TIME_INTERVAL_SECONDS, 0.0);
        assertEquals(timeInterval1.getUnit(), TimeUnit.SECOND);
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        generator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
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
        assertEquals(generator.getAccelerometerBaseNoiseLevel(), 0.0, 0.0);
        final Acceleration baseNoiseLevel1 = generator.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(baseNoiseLevel1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(baseNoiseLevel1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baseNoiseLevel2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel2);
        assertEquals(baseNoiseLevel1, baseNoiseLevel2);
        assertEquals(generator.getAccelerometerBaseNoiseLevelPsd(),
                0.0, 0.0);
        assertEquals(generator.getAccelerometerBaseNoiseLevelRootPsd(),
                0.0, 0.0);
        assertEquals(generator.getThreshold(), 0.0, 0.0);
        final Acceleration threshold1 = generator.getThresholdAsMeasurement();
        assertEquals(threshold1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(threshold1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration threshold2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getThresholdAsMeasurement(threshold2);
        assertEquals(threshold1, threshold2);

        final AngularSpeedTriad avgAngularSpeed1 = new AngularSpeedTriad();
        final AngularSpeedTriad avgAngularSpeed2 = generator.getInitialAvgAngularSpeedTriad();
        final AngularSpeedTriad avgAngularSpeed3 = new AngularSpeedTriad();
        generator.getInitialAvgAngularSpeedTriad(avgAngularSpeed3);
        assertEquals(avgAngularSpeed1, avgAngularSpeed2);
        assertEquals(avgAngularSpeed1, avgAngularSpeed3);

        final AngularSpeedTriad stdAngularSpeed1 = new AngularSpeedTriad();
        final AngularSpeedTriad stdAngularSpeed2 = generator.getInitialAngularSpeedTriadStandardDeviation();
        final AngularSpeedTriad stdAngularSpeed3 = new AngularSpeedTriad();
        generator.getInitialAngularSpeedTriadStandardDeviation(stdAngularSpeed3);
        assertEquals(stdAngularSpeed1, stdAngularSpeed2);
        assertEquals(stdAngularSpeed1, stdAngularSpeed3);

        assertEquals(0.0, generator.getGyroscopeBaseNoiseLevel(), 0.0);
        final AngularSpeed gyroNoiseLevel1 = generator.getGyroscopeBaseNoiseLevelAsMeasurement();
        assertEquals(gyroNoiseLevel1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(gyroNoiseLevel1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed gyroNoiseLevel2 = new AngularSpeed(1.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        generator.getGyroscopeBaseNoiseLevelAsMeasurement(gyroNoiseLevel2);
        assertEquals(gyroNoiseLevel1, gyroNoiseLevel2);
        assertEquals(generator.getGyroscopeBaseNoiseLevelPsd(), 0.0, 0.0);
        assertEquals(generator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0, 0.0);
    }

    @Test
    public void testConstructor2() {
        final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                new AccelerometerAndGyroscopeMeasurementsGenerator(this);

        // check default values
        assertEquals(generator.getTimeInterval(), TIME_INTERVAL_SECONDS,
                0.0);
        final Time timeInterval1 = generator.getTimeIntervalAsTime();
        assertEquals(timeInterval1.getValue().doubleValue(),
                TIME_INTERVAL_SECONDS, 0.0);
        assertEquals(timeInterval1.getUnit(), TimeUnit.SECOND);
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        generator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(generator.getMinStaticSamples(),
                MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES);
        assertEquals(generator.getMaxDynamicSamples(),
                MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES);
        assertSame(this, generator.getListener());
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
        assertEquals(generator.getAccelerometerBaseNoiseLevel(), 0.0, 0.0);
        final Acceleration baseNoiseLevel1 = generator.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(baseNoiseLevel1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(baseNoiseLevel1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baseNoiseLevel2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel2);
        assertEquals(baseNoiseLevel1, baseNoiseLevel2);
        assertEquals(generator.getAccelerometerBaseNoiseLevelPsd(),
                0.0, 0.0);
        assertEquals(generator.getAccelerometerBaseNoiseLevelRootPsd(),
                0.0, 0.0);
        assertEquals(generator.getThreshold(), 0.0, 0.0);
        final Acceleration threshold1 = generator.getThresholdAsMeasurement();
        assertEquals(threshold1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(threshold1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration threshold2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getThresholdAsMeasurement(threshold2);
        assertEquals(threshold1, threshold2);

        final AngularSpeedTriad avgAngularSpeed1 = new AngularSpeedTriad();
        final AngularSpeedTriad avgAngularSpeed2 = generator.getInitialAvgAngularSpeedTriad();
        final AngularSpeedTriad avgAngularSpeed3 = new AngularSpeedTriad();
        generator.getInitialAvgAngularSpeedTriad(avgAngularSpeed3);
        assertEquals(avgAngularSpeed1, avgAngularSpeed2);
        assertEquals(avgAngularSpeed1, avgAngularSpeed3);

        final AngularSpeedTriad stdAngularSpeed1 = new AngularSpeedTriad();
        final AngularSpeedTriad stdAngularSpeed2 = generator.getInitialAngularSpeedTriadStandardDeviation();
        final AngularSpeedTriad stdAngularSpeed3 = new AngularSpeedTriad();
        generator.getInitialAngularSpeedTriadStandardDeviation(stdAngularSpeed3);
        assertEquals(stdAngularSpeed1, stdAngularSpeed2);
        assertEquals(stdAngularSpeed1, stdAngularSpeed3);

        assertEquals(0.0, generator.getGyroscopeBaseNoiseLevel(), 0.0);
        final AngularSpeed gyroNoiseLevel1 = generator.getGyroscopeBaseNoiseLevelAsMeasurement();
        assertEquals(gyroNoiseLevel1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(gyroNoiseLevel1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed gyroNoiseLevel2 = new AngularSpeed(1.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        generator.getGyroscopeBaseNoiseLevelAsMeasurement(gyroNoiseLevel2);
        assertEquals(gyroNoiseLevel1, gyroNoiseLevel2);
        assertEquals(generator.getGyroscopeBaseNoiseLevelPsd(), 0.0, 0.0);
        assertEquals(generator.getGyroscopeBaseNoiseLevelRootPsd(), 0.0, 0.0);
    }

    @Test
    public void testGetSetTimeInterval1() throws LockedException {
        final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                new AccelerometerAndGyroscopeMeasurementsGenerator();

        // check default value
        assertEquals(generator.getTimeInterval(),
                TIME_INTERVAL_SECONDS, 0.0);

        // set new value
        final double timeInterval = 2 * TIME_INTERVAL_SECONDS;
        generator.setTimeInterval(timeInterval);

        // check
        assertEquals(timeInterval, generator.getTimeInterval(), 0.0);

        // Force IllegalArgumentException
        try {
            generator.setTimeInterval(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetTimeInterval2() throws LockedException {
        final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                new AccelerometerAndGyroscopeMeasurementsGenerator();

        // check default value
        final Time timeInterval1 = generator.getTimeIntervalAsTime();
        assertEquals(timeInterval1.getValue().doubleValue(),
                TIME_INTERVAL_SECONDS, 0.0);
        assertEquals(timeInterval1.getUnit(), TimeUnit.SECOND);

        // set new value
        final Time timeInterval2 = new Time(2 * TIME_INTERVAL_SECONDS,
                TimeUnit.SECOND);
        generator.setTimeInterval(timeInterval2);

        // check
        final Time timeInterval3 = generator.getTimeIntervalAsTime();
        final Time timeInterval4 = new Time(1.0, TimeUnit.DAY);
        generator.getTimeIntervalAsTime(timeInterval4);

        assertEquals(timeInterval2, timeInterval3);
        assertEquals(timeInterval2, timeInterval4);

        // Force IllegalArgumentException
        final Time timeInterval5 = new Time(-1.0, TimeUnit.SECOND);
        try {
            generator.setTimeInterval(timeInterval5);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMinStaticSamples() throws LockedException {
        final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                new AccelerometerAndGyroscopeMeasurementsGenerator();

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
        final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                new AccelerometerAndGyroscopeMeasurementsGenerator();

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
        final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                new AccelerometerAndGyroscopeMeasurementsGenerator();

        // check default value
        assertNull(generator.getListener());

        // set new value
        generator.setListener(this);

        // check
        assertSame(generator.getListener(), this);
    }

    @Test
    public void testGetSetWindowSize() throws LockedException {
        final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                new AccelerometerAndGyroscopeMeasurementsGenerator();

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
        final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                new AccelerometerAndGyroscopeMeasurementsGenerator();

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
        final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                new AccelerometerAndGyroscopeMeasurementsGenerator();

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
        final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                new AccelerometerAndGyroscopeMeasurementsGenerator();

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
        final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                new AccelerometerAndGyroscopeMeasurementsGenerator();

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
        final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                new AccelerometerAndGyroscopeMeasurementsGenerator();

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
    public void testProcessCalibrateAndResetWithNoiseMaCommonAxisAndNoGDependentCrossBiases() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, InvalidRotationMatrixException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
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
                    roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                            ecefFrame, ecefFrame);

            reset();
            assertTrue(mAccelerometerMeasurements.isEmpty());
            assertTrue(mGyroscopeMeasurements.isEmpty());
            assertEquals(mInitializationStarted, 0);
            assertEquals(mInitializationCompleted, 0);
            assertEquals(mError, 0);
            assertEquals(mStaticIntervalDetected, 0);
            assertEquals(mDynamicIntervalDetected, 0);
            assertEquals(mGeneratedAccelerometerMeasurement, 0);
            assertEquals(mGeneratedGyroscopeMeasurement, 0);
            assertEquals(mReset, 0);

            final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                    new AccelerometerAndGyroscopeMeasurementsGenerator(this);

            // generate initial static samples
            final int initialStaticSamples = TriadStaticIntervalDetector
                    .DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics,
                    errors, random, 0);

            assertEquals(mInitializationStarted, 1);
            assertEquals(mInitializationCompleted, 1);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final int n = Math.max(numSequences + 1, numMeasurements);

            final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final int dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();

            boolean failed = false;
            int start = initialStaticSamples;
            for (int i = 0; i < n; i++) {
                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics,
                        errors, random, start);
                start += staticPeriodLength;

                assertEquals(mStaticIntervalDetected, i + 1);

                // generate dynamic samples
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        generateDynamicSamples(generator, dynamicPeriodLength,
                                trueKinematics, randomizer, ecefFrame, nedFrame,
                                errors, random, start, false);
                sequences.add(sequence);
                start += dynamicPeriodLength;

                if (mDynamicIntervalDetected != i + 1 || mAccelerometerMeasurements.size() != i + 1
                        || mGyroscopeMeasurements.size() != i) {
                    failed = true;
                    break;
                }

                assertEquals(mDynamicIntervalDetected, i + 1);
                assertEquals(mAccelerometerMeasurements.size(), i + 1);
                assertEquals(mGyroscopeMeasurements.size(), i);
            }

            if (failed) {
                continue;
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

            generator.reset();

            assertEquals(mReset, 1);
            assertEquals(mError, 0);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator trueCalibrator =
                    new EasyGyroscopeCalibrator(sequences,
                            true, false,
                            initialBg, initialMg, initialGg, ba, ma);
            final EasyGyroscopeCalibrator calibrator =
                    new EasyGyroscopeCalibrator(mGyroscopeMeasurements,
                            true, false,
                            initialBg, initialMg, initialGg, ba, ma);

            try {
                trueCalibrator.calibrate();
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBgTrue = trueCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMgTrue = trueCalibrator.getEstimatedMg();
            final Matrix estimatedGgTrue = trueCalibrator.getEstimatedGg();

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            final ECEFGravity gravity = ECEFGravityEstimator
                    .estimateGravityAndReturnNew(ecefFrame);

            KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(
                            gravity.getNorm(), mAccelerometerMeasurements,
                            true);

            try {
                accelerometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

            if (!bg.equals(estimatedBgTrue, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMgTrue, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGgTrue, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBgTrue, ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMgTrue, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGgTrue, 0.0));

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

            if (!ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testProcessCalibrateAndResetSmallNoiseMaGeneralAndNoGDependentCrossBiases() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, InvalidRotationMatrixException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, SMALL_ROOT_PSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
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
                    roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                            ecefFrame, ecefFrame);

            reset();
            assertTrue(mAccelerometerMeasurements.isEmpty());
            assertTrue(mGyroscopeMeasurements.isEmpty());
            assertEquals(mInitializationStarted, 0);
            assertEquals(mInitializationCompleted, 0);
            assertEquals(mError, 0);
            assertEquals(mStaticIntervalDetected, 0);
            assertEquals(mDynamicIntervalDetected, 0);
            assertEquals(mGeneratedAccelerometerMeasurement, 0);
            assertEquals(mGeneratedGyroscopeMeasurement, 0);
            assertEquals(mReset, 0);

            final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                    new AccelerometerAndGyroscopeMeasurementsGenerator(this);

            // generate initial static samples
            final int initialStaticSamples = TriadStaticIntervalDetector
                    .DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics,
                    errors, random, 0);

            assertEquals(mInitializationStarted, 1);
            assertEquals(mInitializationCompleted, 1);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final int n = Math.max(numSequences + 1, numMeasurements);

            final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final int dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();

            int start = initialStaticSamples;
            for (int i = 0; i < n; i++) {
                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics,
                        errors, random, start);
                start += staticPeriodLength;

                assertEquals(mStaticIntervalDetected, i + 1);

                // generate dynamic samples
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        generateDynamicSamples(generator, dynamicPeriodLength,
                                trueKinematics, randomizer, ecefFrame, nedFrame,
                                errors, random, start, false);
                sequences.add(sequence);
                start += dynamicPeriodLength;

                assertEquals(mDynamicIntervalDetected, i + 1);
                assertEquals(mAccelerometerMeasurements.size(), i + 1);
                assertEquals(mGyroscopeMeasurements.size(), i);
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

            generator.reset();

            assertEquals(mReset, 1);
            assertEquals(mError, 0);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator trueCalibrator =
                    new EasyGyroscopeCalibrator(sequences,
                            true, false,
                            initialBg, initialMg, initialGg, ba, ma);
            final EasyGyroscopeCalibrator calibrator =
                    new EasyGyroscopeCalibrator(mGyroscopeMeasurements,
                            true, false,
                            initialBg, initialMg, initialGg, ba, ma);

            try {
                trueCalibrator.calibrate();
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBgTrue = trueCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMgTrue = trueCalibrator.getEstimatedMg();
            final Matrix estimatedGgTrue = trueCalibrator.getEstimatedGg();

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            final ECEFGravity gravity = ECEFGravityEstimator
                    .estimateGravityAndReturnNew(ecefFrame);

            KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(
                            gravity.getNorm(), mAccelerometerMeasurements,
                            false);

            try {
                accelerometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

            if (!bg.equals(estimatedBgTrue, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMgTrue, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGgTrue, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBgTrue, ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMgTrue, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGgTrue, 0.0));

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

            if (!ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testProcessCalibrateAndResetSmallNoiseMaCommonAxisAndNoGDependentCrossBiases() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, InvalidRotationMatrixException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, SMALL_ROOT_PSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
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
                    roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                            ecefFrame, ecefFrame);

            reset();
            assertTrue(mAccelerometerMeasurements.isEmpty());
            assertTrue(mGyroscopeMeasurements.isEmpty());
            assertEquals(mInitializationStarted, 0);
            assertEquals(mInitializationCompleted, 0);
            assertEquals(mError, 0);
            assertEquals(mStaticIntervalDetected, 0);
            assertEquals(mDynamicIntervalDetected, 0);
            assertEquals(mGeneratedAccelerometerMeasurement, 0);
            assertEquals(mGeneratedGyroscopeMeasurement, 0);
            assertEquals(mReset, 0);

            final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                    new AccelerometerAndGyroscopeMeasurementsGenerator(this);

            // generate initial static samples
            final int initialStaticSamples = TriadStaticIntervalDetector
                    .DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics,
                    errors, random, 0);

            assertEquals(mInitializationStarted, 1);
            assertEquals(mInitializationCompleted, 1);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final int n = Math.max(numSequences + 1, numMeasurements);

            final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final int dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();

            int start = initialStaticSamples;
            for (int i = 0; i < n; i++) {
                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics,
                        errors, random, start);
                start += staticPeriodLength;

                assertEquals(mStaticIntervalDetected, i + 1);

                // generate dynamic samples
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        generateDynamicSamples(generator, dynamicPeriodLength,
                                trueKinematics, randomizer, ecefFrame, nedFrame,
                                errors, random, start, false);
                sequences.add(sequence);
                start += dynamicPeriodLength;

                assertEquals(mDynamicIntervalDetected, i + 1);
                assertEquals(mAccelerometerMeasurements.size(), i + 1);
                assertEquals(mGyroscopeMeasurements.size(), i);
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

            generator.reset();

            assertEquals(mReset, 1);
            assertEquals(mError, 0);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator trueCalibrator =
                    new EasyGyroscopeCalibrator(sequences,
                            true, false,
                            initialBg, initialMg, initialGg, ba, ma);
            final EasyGyroscopeCalibrator calibrator =
                    new EasyGyroscopeCalibrator(mGyroscopeMeasurements,
                            true, false,
                            initialBg, initialMg, initialGg, ba, ma);

            try {
                trueCalibrator.calibrate();
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBgTrue = trueCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMgTrue = trueCalibrator.getEstimatedMg();
            final Matrix estimatedGgTrue = trueCalibrator.getEstimatedGg();

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            final ECEFGravity gravity = ECEFGravityEstimator
                    .estimateGravityAndReturnNew(ecefFrame);

            KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(
                            gravity.getNorm(), mAccelerometerMeasurements,
                            true);

            try {
                accelerometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

            if (!bg.equals(estimatedBgTrue, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMgTrue, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGgTrue, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBgTrue, SMALL_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMgTrue, SMALL_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGgTrue, 0.0));

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
    public void testProcessCalibrateAndResetSmallNoiseMaGeneralAndWithGDependentCrossBiases() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, InvalidRotationMatrixException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, SMALL_ROOT_PSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
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
                    roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                            ecefFrame, ecefFrame);

            reset();
            assertTrue(mAccelerometerMeasurements.isEmpty());
            assertTrue(mGyroscopeMeasurements.isEmpty());
            assertEquals(mInitializationStarted, 0);
            assertEquals(mInitializationCompleted, 0);
            assertEquals(mError, 0);
            assertEquals(mStaticIntervalDetected, 0);
            assertEquals(mDynamicIntervalDetected, 0);
            assertEquals(mGeneratedAccelerometerMeasurement, 0);
            assertEquals(mGeneratedGyroscopeMeasurement, 0);
            assertEquals(mReset, 0);

            final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                    new AccelerometerAndGyroscopeMeasurementsGenerator(this);

            // generate initial static samples
            final int initialStaticSamples = TriadStaticIntervalDetector
                    .DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics,
                    errors, random, 0);

            assertEquals(mInitializationStarted, 1);
            assertEquals(mInitializationCompleted, 1);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS_AND_CROSS_BIASES;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final int n = Math.max(numSequences + 1, numMeasurements);

            final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final int dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();

            int start = initialStaticSamples;
            for (int i = 0; i < n; i++) {
                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics,
                        errors, random, start);
                start += staticPeriodLength;

                assertEquals(mStaticIntervalDetected, i + 1);

                // generate dynamic samples
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        generateDynamicSamples(generator, dynamicPeriodLength,
                                trueKinematics, randomizer, ecefFrame, nedFrame,
                                errors, random, start, false);
                sequences.add(sequence);
                start += dynamicPeriodLength;

                assertEquals(mDynamicIntervalDetected, i + 1);
                assertEquals(mAccelerometerMeasurements.size(), i + 1);
                assertEquals(mGyroscopeMeasurements.size(), i);
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

            generator.reset();

            assertEquals(mReset, 1);
            assertEquals(mError, 0);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator trueCalibrator =
                    new EasyGyroscopeCalibrator(sequences,
                            true, true,
                            initialBg, initialMg, initialGg, ba, ma);
            final EasyGyroscopeCalibrator calibrator =
                    new EasyGyroscopeCalibrator(mGyroscopeMeasurements,
                            true, true,
                            initialBg, initialMg, initialGg, ba, ma);

            try {
                trueCalibrator.calibrate();
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBgTrue = trueCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMgTrue = trueCalibrator.getEstimatedMg();
            final Matrix estimatedGgTrue = trueCalibrator.getEstimatedGg();

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            final ECEFGravity gravity = ECEFGravityEstimator
                    .estimateGravityAndReturnNew(ecefFrame);

            KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(
                            gravity.getNorm(), mAccelerometerMeasurements,
                            false);

            try {
                accelerometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

            if (!bg.equals(estimatedBgTrue, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMgTrue, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGgTrue, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBgTrue, SMALL_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMgTrue, SMALL_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGgTrue, SMALL_ABSOLUTE_ERROR));

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

            if (!ba.equals(estimatedBa, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, SMALL_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testProcessSkipStaticInterval() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            InvalidRotationMatrixException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

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
                roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                        ecefFrame, ecefFrame);

        reset();
        assertTrue(mAccelerometerMeasurements.isEmpty());
        assertTrue(mGyroscopeMeasurements.isEmpty());
        assertEquals(mInitializationStarted, 0);
        assertEquals(mInitializationCompleted, 0);
        assertEquals(mError, 0);
        assertEquals(mStaticIntervalDetected, 0);
        assertEquals(mDynamicIntervalDetected, 0);
        assertEquals(mGeneratedAccelerometerMeasurement, 0);
        assertEquals(mGeneratedGyroscopeMeasurement, 0);
        assertEquals(mReset, 0);

        final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                new AccelerometerAndGyroscopeMeasurementsGenerator(this);

        // generate initial static samples
        final int initialStaticSamples = TriadStaticIntervalDetector
                .DEFAULT_INITIAL_STATIC_SAMPLES;
        generateStaticSamples(generator, initialStaticSamples, trueKinematics,
                errors, random, 0);

        assertEquals(mInitializationStarted, 1);
        assertEquals(mInitializationCompleted, 1);

        final int staticPeriodLength = generator.getMinStaticSamples() / 2;
        final int dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

        int start = initialStaticSamples;
        // generate static samples
        generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors,
                random, start);
        start += staticPeriodLength;

        assertEquals(mStaticIntervalDetected, 1);

        // generate dynamic samples
        assertNotNull(generateDynamicSamples(generator, dynamicPeriodLength,
                trueKinematics, randomizer, ecefFrame, nedFrame, errors,
                random, start, false));

        assertEquals(mDynamicIntervalDetected, 1);
        assertEquals(mStaticIntervalSkipped, 1);
    }

    @Test
    public void testProcessSkipDynamicInterval() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            InvalidRotationMatrixException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMaCommonAxis();
            final Matrix mg = generateMg();
            final Matrix gg = new Matrix(3, 3);

            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                    gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

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
                    roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                            ecefFrame, ecefFrame);

            reset();
            assertTrue(mAccelerometerMeasurements.isEmpty());
            assertTrue(mGyroscopeMeasurements.isEmpty());
            assertEquals(mInitializationStarted, 0);
            assertEquals(mInitializationCompleted, 0);
            assertEquals(mError, 0);
            assertEquals(mStaticIntervalDetected, 0);
            assertEquals(mDynamicIntervalDetected, 0);
            assertEquals(mGeneratedAccelerometerMeasurement, 0);
            assertEquals(mGeneratedGyroscopeMeasurement, 0);
            assertEquals(mReset, 0);

            final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                    new AccelerometerAndGyroscopeMeasurementsGenerator(this);

            // generate initial static samples
            final int initialStaticSamples = TriadStaticIntervalDetector
                    .DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics,
                    errors, random, 0);

            assertEquals(mInitializationStarted, 1);
            assertEquals(mInitializationCompleted, 1);

            final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final int dynamicPeriodLength = 2 * generator.getMaxDynamicSamples();

            int start = initialStaticSamples;
            // generate static samples
            generateStaticSamples(generator, staticPeriodLength, trueKinematics,
                    errors, random, start);
            start += staticPeriodLength;

            assertEquals(mStaticIntervalDetected, 1);

            // generate dynamic samples
            assertNotNull(generateDynamicSamples(generator, dynamicPeriodLength,
                    trueKinematics, randomizer, ecefFrame, nedFrame, errors,
                    random, start, false));

            if (mDynamicIntervalDetected != 1) {
                continue;
            }
            assertEquals(mDynamicIntervalDetected, 1);
            assertEquals(mDynamicIntervalSkipped, 1);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testProcessErrorWithExcessiveOverallNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

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
                roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                        ecefFrame, ecefFrame);

        reset();
        assertTrue(mAccelerometerMeasurements.isEmpty());
        assertTrue(mGyroscopeMeasurements.isEmpty());
        assertEquals(mInitializationStarted, 0);
        assertEquals(mInitializationCompleted, 0);
        assertEquals(mError, 0);
        assertEquals(mStaticIntervalDetected, 0);
        assertEquals(mDynamicIntervalDetected, 0);
        assertEquals(mGeneratedAccelerometerMeasurement, 0);
        assertEquals(mGeneratedGyroscopeMeasurement, 0);
        assertEquals(mReset, 0);

        final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                new AccelerometerAndGyroscopeMeasurementsGenerator(this);
        generator.setBaseNoiseLevelAbsoluteThreshold(Double.MIN_VALUE);


        // generate initial static samples
        final int initialStaticSamples = TriadStaticIntervalDetector
                .DEFAULT_INITIAL_STATIC_SAMPLES;
        generateStaticSamples(generator, initialStaticSamples, trueKinematics,
                errors, random, 0);

        assertEquals(mInitializationStarted, 1);
        assertEquals(mInitializationCompleted, 0);
        assertEquals(mError, 1);

        final TimedBodyKinematics timeKinematics = new TimedBodyKinematics();
        timeKinematics.setKinematics(trueKinematics);
        assertFalse(generator.process(timeKinematics));

        generator.reset();

        assertEquals(mReset, 1);

        assertTrue(generator.process(timeKinematics));
    }

    @Test
    public void testProcessCalibrateAndResetSmallNoiseWithRotationAndPositionChangeMaCommonAxisAndNoGDependentCrossBiases()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, InvalidRotationMatrixException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, SMALL_ROOT_PSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
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
                    roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            // compute ground-truth kinematics that should be generated at provided
            // position, velocity and orientation
            final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                            ecefFrame, ecefFrame);

            reset();
            assertTrue(mAccelerometerMeasurements.isEmpty());
            assertTrue(mGyroscopeMeasurements.isEmpty());
            assertEquals(mInitializationStarted, 0);
            assertEquals(mInitializationCompleted, 0);
            assertEquals(mError, 0);
            assertEquals(mStaticIntervalDetected, 0);
            assertEquals(mDynamicIntervalDetected, 0);
            assertEquals(mGeneratedAccelerometerMeasurement, 0);
            assertEquals(mGeneratedGyroscopeMeasurement, 0);
            assertEquals(mReset, 0);

            final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                    new AccelerometerAndGyroscopeMeasurementsGenerator(this);


            // generate initial static samples
            final int initialStaticSamples = TriadStaticIntervalDetector
                    .DEFAULT_INITIAL_STATIC_SAMPLES;
            generateStaticSamples(generator, initialStaticSamples, trueKinematics,
                    errors, random, 0);

            assertEquals(mInitializationStarted, 1);
            assertEquals(mInitializationCompleted, 1);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMON_Z_AXIS;
            final int n = Math.max(numSequences + 1, numMeasurements);

            final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
            final int dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();

            int start = initialStaticSamples;
            for (int i = 0; i < n; i++) {
                // generate static samples
                generateStaticSamples(generator, staticPeriodLength, trueKinematics,
                        errors, random, start);
                start += staticPeriodLength;

                assertEquals(mStaticIntervalDetected, i + 1);

                // generate dynamic samples
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        generateDynamicSamples(generator, dynamicPeriodLength,
                                trueKinematics, randomizer, ecefFrame, nedFrame,
                                errors, random, start, true);
                sequences.add(sequence);
                start += dynamicPeriodLength;

                assertEquals(mDynamicIntervalDetected, i + 1);
                assertEquals(mAccelerometerMeasurements.size(), i + 1);
                assertEquals(mGyroscopeMeasurements.size(), i);
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

            generator.reset();

            assertEquals(mReset, 1);
            assertEquals(mError, 0);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator trueCalibrator =
                    new EasyGyroscopeCalibrator(sequences,
                            true, false,
                            initialBg, initialMg, initialGg, ba, ma);
            final EasyGyroscopeCalibrator calibrator =
                    new EasyGyroscopeCalibrator(mGyroscopeMeasurements,
                            true, false,
                            initialBg, initialMg, initialGg, ba, ma);

            try {
                trueCalibrator.calibrate();
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBgTrue = trueCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMgTrue = trueCalibrator.getEstimatedMg();
            final Matrix estimatedGgTrue = trueCalibrator.getEstimatedGg();

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            final ECEFGravity gravity = ECEFGravityEstimator
                    .estimateGravityAndReturnNew(ecefFrame);

            KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(
                            gravity.getNorm(), mAccelerometerMeasurements,
                            true);

            try {
                accelerometerCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();


            if (!bg.equals(estimatedBgTrue, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMgTrue, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGgTrue, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBgTrue, SMALL_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMgTrue, SMALL_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGgTrue, 0.0));

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

    @Override
    public void onInitializationStarted(
            final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
        mInitializationStarted++;
        checkLocked(generator);

        assertEquals(generator.getStatus(),
                TriadStaticIntervalDetector.Status.INITIALIZING);
    }

    @Override
    public void onInitializationCompleted(
            final AccelerometerAndGyroscopeMeasurementsGenerator generator,
            final double accelerometerBaseNoiseLevel) {
        mInitializationCompleted++;
        checkLocked(generator);

        assertTrue(accelerometerBaseNoiseLevel > 0.0);
        assertEquals(accelerometerBaseNoiseLevel, generator.getAccelerometerBaseNoiseLevel(), 0.0);
        final Acceleration baseNoiseLevel1 = generator.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(baseNoiseLevel1.getValue().doubleValue(), accelerometerBaseNoiseLevel, 0.0);
        assertEquals(baseNoiseLevel1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baseNoiseLevel2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getAccelerometerBaseNoiseLevelAsMeasurement(baseNoiseLevel2);
        assertEquals(baseNoiseLevel1, baseNoiseLevel2);
        final double sqrtTimeInterval = Math.sqrt(generator.getTimeInterval());
        assertEquals(accelerometerBaseNoiseLevel * sqrtTimeInterval,
                generator.getAccelerometerBaseNoiseLevelRootPsd(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(generator.getAccelerometerBaseNoiseLevelPsd(),
                Math.pow(generator.getAccelerometerBaseNoiseLevelRootPsd(), 2.0),
                SMALL_ABSOLUTE_ERROR);

        assertTrue(generator.getThreshold() > 0.0);
        final Acceleration threshold1 = generator.getThresholdAsMeasurement();
        assertEquals(threshold1.getValue().doubleValue(), generator.getThreshold(),
                0.0);
        assertEquals(threshold1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration threshold2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        generator.getThresholdAsMeasurement(threshold2);
        assertEquals(threshold1, threshold2);

        final AngularSpeedTriad avgAngularSpeed1 = generator.getInitialAvgAngularSpeedTriad();
        final AngularSpeedTriad avgAngularSpeed2 = new AngularSpeedTriad();
        generator.getInitialAvgAngularSpeedTriad(avgAngularSpeed2);
        assertEquals(avgAngularSpeed1, avgAngularSpeed2);

        final AngularSpeedTriad stdAngularSpeed1 = generator.getInitialAngularSpeedTriadStandardDeviation();
        final AngularSpeedTriad stdAngularSpeed2 = new AngularSpeedTriad();
        generator.getInitialAngularSpeedTriadStandardDeviation(stdAngularSpeed2);
        assertEquals(stdAngularSpeed1, stdAngularSpeed2);

        final double gyroNoiseLevel = generator.getGyroscopeBaseNoiseLevel();
        assertTrue(gyroNoiseLevel > 0.0);
        final AngularSpeed gyroNoiseLevel1 = generator.getGyroscopeBaseNoiseLevelAsMeasurement();
        assertEquals(gyroNoiseLevel1.getValue().doubleValue(), gyroNoiseLevel, 0.0);
        assertEquals(gyroNoiseLevel1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed gyroNoiseLevel2 = new AngularSpeed(
                1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        generator.getGyroscopeBaseNoiseLevelAsMeasurement(gyroNoiseLevel2);
        assertEquals(gyroNoiseLevel1, gyroNoiseLevel2);

        assertEquals(stdAngularSpeed1.getNorm(), gyroNoiseLevel, 0.0);

        assertEquals(gyroNoiseLevel * sqrtTimeInterval,
                generator.getGyroscopeBaseNoiseLevelRootPsd(),
                SMALL_ABSOLUTE_ERROR);
        assertEquals(generator.getGyroscopeBaseNoiseLevelPsd(),
                Math.pow(generator.getGyroscopeBaseNoiseLevelRootPsd(), 2.0),
                SMALL_ABSOLUTE_ERROR);
    }

    @Override
    public void onError(
            final AccelerometerAndGyroscopeMeasurementsGenerator generator,
            final TriadStaticIntervalDetector.ErrorReason reason) {
        mError++;
        checkLocked(generator);

        assertEquals(generator.getStatus(),
                TriadStaticIntervalDetector.Status.FAILED);
    }

    @Override
    public void onStaticIntervalDetected(
            final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
        mStaticIntervalDetected++;
        checkLocked(generator);

        assertEquals(generator.getStatus(),
                TriadStaticIntervalDetector.Status.STATIC_INTERVAL);
    }

    @Override
    public void onDynamicIntervalDetected(
            final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
        mDynamicIntervalDetected++;
        checkLocked(generator);

        assertEquals(generator.getStatus(),
                TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL);
    }

    @Override
    public void onStaticIntervalSkipped(
            final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
        mStaticIntervalSkipped++;
        checkLocked(generator);

        assertEquals(generator.getStatus(),
                TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL);
    }

    @Override
    public void onDynamicIntervalSkipped(
            final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
        mDynamicIntervalSkipped++;
        checkLocked(generator);

        assertEquals(generator.getStatus(),
                TriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL);
    }

    @Override
    public void onGeneratedAccelerometerMeasurement(
            final AccelerometerAndGyroscopeMeasurementsGenerator generator,
            final StandardDeviationBodyKinematics measurement) {
        mGeneratedAccelerometerMeasurement++;
        mAccelerometerMeasurements.add(measurement);
        checkLocked(generator);
    }

    @Override
    public void onGeneratedGyroscopeMeasurement(
            final AccelerometerAndGyroscopeMeasurementsGenerator generator,
            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> measurement) {
        mGeneratedGyroscopeMeasurement++;
        mGyroscopeMeasurements.add(measurement);
        checkLocked(generator);
    }

    @Override
    public void onReset(
            final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
        mReset++;

        assertEquals(generator.getStatus(),
                TriadStaticIntervalDetector.Status.IDLE);
    }

    private void reset() {
        mGyroscopeMeasurements.clear();
        mAccelerometerMeasurements.clear();

        mInitializationStarted = 0;
        mInitializationCompleted = 0;
        mError = 0;
        mStaticIntervalDetected = 0;
        mDynamicIntervalDetected = 0;
        mStaticIntervalSkipped = 0;
        mDynamicIntervalSkipped = 0;
        mGeneratedAccelerometerMeasurement = 0;
        mGeneratedGyroscopeMeasurement = 0;
        mReset = 0;
    }

    private void checkLocked(
            final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
        assertTrue(generator.isRunning());
        try {
            generator.setTimeInterval(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        final Time timeInterval = new Time(1.0, TimeUnit.SECOND);
        try {
            generator.setTimeInterval(timeInterval);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
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
            final AccelerometerAndGyroscopeMeasurementsGenerator generator,
            final int numSamples,
            final BodyKinematics trueKinematics,
            final IMUErrors errors,
            final Random random,
            final int startSample)
            throws LockedException {

        final TimedBodyKinematics timedMeasuredKinematics = new TimedBodyKinematics();
        final BodyKinematics measuredKinematics = new BodyKinematics();
        for (int i = 0, j = startSample; i < numSamples; i++, j++) {

            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);

            timedMeasuredKinematics.setKinematics(measuredKinematics);
            timedMeasuredKinematics.setTimestampSeconds(
                    j * TIME_INTERVAL_SECONDS);

            assertTrue(generator.process(timedMeasuredKinematics));
        }
    }

    @SuppressWarnings("SameParameterValue")
    private BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> generateDynamicSamples(
            final AccelerometerAndGyroscopeMeasurementsGenerator generator,
            final int numSamples,
            final BodyKinematics trueKinematics,
            final UniformRandomizer randomizer,
            final ECEFFrame ecefFrame,
            final NEDFrame nedFrame,
            final IMUErrors errors,
            final Random random,
            final int startSample,
            final boolean changePosition)
            throws InvalidSourceAndDestinationFrameTypeException, LockedException, InvalidRotationMatrixException {

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

        final List<StandardDeviationTimedBodyKinematics> measuredTimedKinematicsList =
                new ArrayList<>();

        final TimedBodyKinematics timedMeasuredKinematics = new TimedBodyKinematics();
        final BodyKinematics measuredKinematics = new BodyKinematics();

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
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);

            timedMeasuredKinematics.setKinematics(measuredKinematics);
            timedMeasuredKinematics.setTimestampSeconds(timestampSeconds);

            assertTrue(generator.process(timedMeasuredKinematics));

            final StandardDeviationTimedBodyKinematics trueTimedKinematics =
                    new StandardDeviationTimedBodyKinematics(
                            new BodyKinematics(trueKinematics), timestampSeconds,
                            specificForceStandardDeviation,
                            angularRateStandardDeviation);
            trueTimedKinematicsList.add(trueTimedKinematics);

            final StandardDeviationTimedBodyKinematics measuredTimedKinematics =
                    new StandardDeviationTimedBodyKinematics(
                            new BodyKinematics(measuredKinematics), timestampSeconds,
                            specificForceStandardDeviation,
                            angularRateStandardDeviation);
            measuredTimedKinematicsList.add(measuredTimedKinematics);

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

        final BodyKinematics measuredAfterGravityKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS,
                        trueKinematics, errors, random);
        final double afterMeanFx = measuredAfterGravityKinematics.getFx();
        final double afterMeanFy = measuredAfterGravityKinematics.getFy();
        final double afterMeanFz = measuredAfterGravityKinematics.getFz();

        sequence.setItems(measuredTimedKinematicsList);
        sequence.setAfterMeanSpecificForceCoordinates(
                afterMeanFx, afterMeanFy, afterMeanFz);

        return sequence;
    }

    // This is required to simulate a smooth transition of values during
    // dynamic period, to avoid a sudden rotation or translation and simulate
    // a more natural behaviour.
    private double interpolate(final double value, final double progress) {
        return -2.0 * (Math.abs(progress - 0.5) - 0.5) * value;
    }
}
