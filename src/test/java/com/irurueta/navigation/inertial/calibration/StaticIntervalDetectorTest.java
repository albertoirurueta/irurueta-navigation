package com.irurueta.navigation.inertial.calibration;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class StaticIntervalDetectorTest implements StaticIntervalDetectorListener {

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

    private static final double MIN_DELTA_POS_METERS = -0.01;
    private static final double MAX_DELTA_POS_METERS = 0.01;
    private static final double MIN_DELTA_ANGLE_DEGREES = -2.0;
    private static final double MAX_DELTA_ANGLE_DEGREES = 2.0;

    private static final double ABSOLUTE_ERROR = 1e-1;

    private int mInitializationStarted;

    private int mInitializationCompleted;

    private int mError;

    private int mStaticIntervalDetected;

    private int mDynamicIntervalDetected;

    private int mReset;

    private double mErrorAccumulatedNoiseLevel;

    private double mErrorInstantaneousNoiseLevel;

    private StaticIntervalDetector.ErrorReason mErrorReason;

    @Test
    public void testConstructor1() {
        final StaticIntervalDetector detector = new StaticIntervalDetector();

        assertEquals(detector.getWindowSize(),
                StaticIntervalDetector.DEFAULT_WINDOW_SIZE);
        assertEquals(detector.getInitialStaticSamples(),
                StaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES);
        assertEquals(detector.getThresholdFactor(),
                StaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, 0.0);
        assertEquals(detector.getInstantaneousNoiseLevelFactor(),
                StaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                0.0);
        assertEquals(detector.getBaseNoiseLevelAbsoluteThreshold(),
                StaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                0.0);
        assertNull(detector.getListener());
        assertEquals(detector.getStatus(), StaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        final Acceleration a1 = detector.getBaseNoiseLevelAsAcceleration();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsAcceleration(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        final Acceleration a3 = detector.getThresholdAsAcceleration();
        assertEquals(a3.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a3.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a4 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getThresholdAsAcceleration(a4);
        assertEquals(a3, a4);
        assertFalse(detector.isRunning());
        assertEquals(detector.getProcessedSamples(), 0);
    }

    @Test
    public void testConstructor2() {
        final StaticIntervalDetector detector = new StaticIntervalDetector(
                this);

        assertEquals(detector.getWindowSize(),
                StaticIntervalDetector.DEFAULT_WINDOW_SIZE);
        assertEquals(detector.getInitialStaticSamples(),
                StaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES);
        assertEquals(detector.getThresholdFactor(),
                StaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, 0.0);
        assertEquals(detector.getInstantaneousNoiseLevelFactor(),
                StaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                0.0);
        assertEquals(detector.getBaseNoiseLevelAbsoluteThreshold(),
                StaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                0.0);
        assertSame(detector.getListener(), this);
        assertEquals(detector.getStatus(), StaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        final Acceleration a1 = detector.getBaseNoiseLevelAsAcceleration();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsAcceleration(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        final Acceleration a3 = detector.getThresholdAsAcceleration();
        assertEquals(a3.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a3.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a4 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getThresholdAsAcceleration(a4);
        assertEquals(a3, a4);
        assertFalse(detector.isRunning());
        assertEquals(detector.getProcessedSamples(), 0);
    }

    @Test
    public void testGetSetWindowSize() throws LockedException {
        final StaticIntervalDetector detector = new StaticIntervalDetector();

        // check default value
        assertEquals(detector.getWindowSize(),
                StaticIntervalDetector.DEFAULT_WINDOW_SIZE);

        // set new value
        detector.setWindowSize(2);

        // check
        assertEquals(detector.getWindowSize(), 2);

        // Force IllegalArgumentException
        try {
            detector.setWindowSize(1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetInitialStaticSamples() throws LockedException {
        final StaticIntervalDetector detector = new StaticIntervalDetector();

        // check default value
        assertEquals(detector.getInitialStaticSamples(),
                StaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES);

        // set new value
        detector.setInitialStaticSamples(2);

        // check
        assertEquals(detector.getInitialStaticSamples(), 2);

        // Force IllegalArgumentException
        try {
            detector.setInitialStaticSamples(1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetThresholdFactor() throws LockedException {
        final StaticIntervalDetector detector = new StaticIntervalDetector();

        // check default value
        assertEquals(detector.getThresholdFactor(),
                StaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, 0.0);

        // set new value
        detector.setThresholdFactor(1.0);

        // check
        assertEquals(detector.getThresholdFactor(), 1.0, 0.0);

        // Force IllegalArgumentException
        try {
            detector.setThresholdFactor(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetInstantaneousNoiseLevelFactor() throws LockedException {
        final StaticIntervalDetector detector = new StaticIntervalDetector();

        // check default value
        assertEquals(detector.getInstantaneousNoiseLevelFactor(),
                StaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                0.0);

        // set new value
        detector.setInstantaneousNoiseLevelFactor(1.0);

        // check
        assertEquals(detector.getInstantaneousNoiseLevelFactor(),
                1.0, 0.0);

        // Force IllegalArgumentException
        try {
            detector.setInstantaneousNoiseLevelFactor(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetBaseNoiseLevelAbsoluteThreshold()
            throws LockedException {
        final StaticIntervalDetector detector = new StaticIntervalDetector();

        // check default value
        assertEquals(detector.getBaseNoiseLevelAbsoluteThreshold(),
                StaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                0.0);

        // set new value
        detector.setBaseNoiseLevelAbsoluteThreshold(1.0);

        // check
        assertEquals(detector.getBaseNoiseLevelAbsoluteThreshold(),
                1.0, 0.0);

        // Force IllegalArgumentException
        try {
            detector.setBaseNoiseLevelAbsoluteThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetBaseNoiseLevelAbsoluteThresholdAsAcceleration()
            throws LockedException {

        final StaticIntervalDetector detector = new StaticIntervalDetector();

        // check default value
        assertEquals(detector.getBaseNoiseLevelAbsoluteThreshold(),
                StaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                0.0);

        final Acceleration a1 = detector.getBaseNoiseLevelAbsoluteThresholdAsAcceleration();
        assertEquals(a1.getValue().doubleValue(),
                StaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // set new value
        final Acceleration a2 = new Acceleration(1.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.setBaseNoiseLevelAbsoluteThreshold(a2);

        // check
        final Acceleration a3 = detector.getBaseNoiseLevelAbsoluteThresholdAsAcceleration();
        final Acceleration a4 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAbsoluteThresholdAsAcceleration(a4);
        assertEquals(a2, a3);
        assertEquals(a2, a4);
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final StaticIntervalDetector detector = new StaticIntervalDetector();

        // check default value
        assertNull(detector.getListener());

        // set new value
        detector.setListener(this);

        // check
        assertSame(detector.getListener(), this);
    }

    @Test
    public void testProcessAccelerationTriadWithSuccessfulInitializationStaticAndDynamicPeriodAndReset1()
            throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
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

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

        reset();
        assertEquals(mInitializationStarted, 0);
        assertEquals(mInitializationCompleted, 0);
        assertEquals(mError, 0);
        assertEquals(mStaticIntervalDetected, 0);
        assertEquals(mDynamicIntervalDetected, 0);
        assertEquals(mReset, 0);

        final StaticIntervalDetector detector =
                new StaticIntervalDetector(this);

        assertEquals(detector.getStatus(), StaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        Acceleration a1 = detector.getBaseNoiseLevelAsAcceleration();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsAcceleration(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        a1 = detector.getThresholdAsAcceleration();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getThresholdAsAcceleration(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getProcessedSamples(), 0);

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        final BodyKinematics measuredKinematics = new BodyKinematics();
        final AccelerationTriad triad = new AccelerationTriad();
        for (int i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.processAccelerationTriad(triad));
        }

        assertEquals(mInitializationStarted, 1);
        assertEquals(mInitializationCompleted, 1);
        assertEquals(detector.getStatus(),
                StaticIntervalDetector.Status.INITIALIZATION_COMPLETED);
        assertTrue(detector.getBaseNoiseLevel() > 0.0);
        a1 = detector.getBaseNoiseLevelAsAcceleration();
        assertEquals(a1.getValue().doubleValue(), detector.getBaseNoiseLevel(),
                0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsAcceleration(a2);
        assertEquals(a1, a2);
        assertTrue(detector.getThreshold() > 0.0);
        assertEquals(detector.getThreshold(),
                detector.getBaseNoiseLevel() * detector.getThresholdFactor(),
                0.0);
        a1 = detector.getThresholdAsAcceleration();
        assertEquals(a1.getValue().doubleValue(), detector.getThreshold(),
                0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getThresholdAsAcceleration(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getProcessedSamples(), initialStaticSamples);

        // keep adding static samples for twice the window size
        int periodLength = 2 * detector.getWindowSize();
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.processAccelerationTriad(triad));
        }

        assertEquals(mStaticIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                StaticIntervalDetector.Status.STATIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + periodLength);

        final AccelerationTriad lastStaticTriad =
                trueKinematics.getSpecificForceTriad();

        // add dynamic samples for twice the window size
        final double deltaX = randomizer.nextDouble(
                MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final double deltaY = randomizer.nextDouble(
                MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final double deltaZ = randomizer.nextDouble(
                MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);

        final double deltaRoll = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaPitch = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaYaw = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final double ecefX = ecefFrame.getX();
        final double ecefY = ecefFrame.getY();
        final double ecefZ = ecefFrame.getZ();

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

        for (int i = 0; i < periodLength; i++) {
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
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.processAccelerationTriad(triad));

            oldNedFrame.copyFrom(newNedFrame);
            oldEcefFrame.copyFrom(newEcefFrame);
            oldRoll = newRoll;
            oldPitch = newPitch;
            oldYaw = newYaw;
            oldEcefX = oldEcefFrame.getX();
            oldEcefY = oldEcefFrame.getY();
            oldEcefZ = oldEcefFrame.getZ();
        }

        assertEquals(mStaticIntervalDetected, 1);
        assertEquals(mDynamicIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                StaticIntervalDetector.Status.DYNAMIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + 2 * periodLength);

        // check that when switching to dynamic period, estimated average
        // specific force from last static period is approximately equal to the
        // true value
        assertEquals(lastStaticTriad.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(lastStaticTriad.getValueX(),
                detector.getAvgSpecificForceX(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueY(),
                detector.getAvgSpecificForceY(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueZ(),
                detector.getAvgSpecificForceZ(), ABSOLUTE_ERROR);
        assertTrue(lastStaticTriad.getMeasurementX().equals(
                detector.getAvgSpecificForceXAsAcceleration(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementY().equals(
                detector.getAvgSpecificForceYAsAcceleration(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementZ().equals(
                detector.getAvgSpecificForceZAsAcceleration(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.equals(detector.getAvgSpecificForce(),
                ABSOLUTE_ERROR));


        // keep adding static samples for twice the window size to last
        // true kinematics
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.processAccelerationTriad(triad));
        }

        assertEquals(mStaticIntervalDetected, 2);
        assertEquals(mDynamicIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                StaticIntervalDetector.Status.STATIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + 3 * periodLength);

        // reset
        detector.reset();

        assertEquals(mReset, 1);
        assertEquals(detector.getStatus(), StaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        a1 = detector.getBaseNoiseLevelAsAcceleration();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsAcceleration(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        a1 = detector.getThresholdAsAcceleration();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getThresholdAsAcceleration(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getProcessedSamples(), 0);
    }

    @Test
    public void testProcessAccelerationTriadWithSuccessfulInitializationStaticAndDynamicPeriodAndReset2()
            throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
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

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

        reset();
        assertEquals(mInitializationStarted, 0);
        assertEquals(mInitializationCompleted, 0);
        assertEquals(mError, 0);
        assertEquals(mStaticIntervalDetected, 0);
        assertEquals(mDynamicIntervalDetected, 0);
        assertEquals(mReset, 0);

        final StaticIntervalDetector detector =
                new StaticIntervalDetector(this);

        assertEquals(detector.getStatus(), StaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        Acceleration a1 = detector.getBaseNoiseLevelAsAcceleration();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsAcceleration(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        a1 = detector.getThresholdAsAcceleration();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getThresholdAsAcceleration(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getProcessedSamples(), 0);

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        final BodyKinematics measuredKinematics = new BodyKinematics();
        final AccelerationTriad triad = new AccelerationTriad();
        final Acceleration aX = new Acceleration(
                0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration aY = new Acceleration(
                0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration aZ = new Acceleration(
                0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        for (int i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);
            triad.getMeasurementX(aX);
            triad.getMeasurementY(aY);
            triad.getMeasurementZ(aZ);

            assertTrue(detector.processAccelerationTriad(aX, aY, aZ));
        }

        assertEquals(mInitializationStarted, 1);
        assertEquals(mInitializationCompleted, 1);
        assertEquals(detector.getStatus(),
                StaticIntervalDetector.Status.INITIALIZATION_COMPLETED);
        assertTrue(detector.getBaseNoiseLevel() > 0.0);
        a1 = detector.getBaseNoiseLevelAsAcceleration();
        assertEquals(a1.getValue().doubleValue(), detector.getBaseNoiseLevel(),
                0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsAcceleration(a2);
        assertEquals(a1, a2);
        assertTrue(detector.getThreshold() > 0.0);
        assertEquals(detector.getThreshold(),
                detector.getBaseNoiseLevel() * detector.getThresholdFactor(),
                0.0);
        a1 = detector.getThresholdAsAcceleration();
        assertEquals(a1.getValue().doubleValue(), detector.getThreshold(),
                0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getThresholdAsAcceleration(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getProcessedSamples(), initialStaticSamples);

        // keep adding static samples for twice the window size
        int periodLength = 2 * detector.getWindowSize();
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);
            triad.getMeasurementX(aX);
            triad.getMeasurementY(aY);
            triad.getMeasurementZ(aZ);

            assertTrue(detector.processAccelerationTriad(aX, aY, aZ));
        }

        assertEquals(mStaticIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                StaticIntervalDetector.Status.STATIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + periodLength);

        final AccelerationTriad lastStaticTriad =
                trueKinematics.getSpecificForceTriad();

        // add dynamic samples for twice the window size
        final double deltaX = randomizer.nextDouble(
                MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final double deltaY = randomizer.nextDouble(
                MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final double deltaZ = randomizer.nextDouble(
                MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);

        final double deltaRoll = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaPitch = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaYaw = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final double ecefX = ecefFrame.getX();
        final double ecefY = ecefFrame.getY();
        final double ecefZ = ecefFrame.getZ();

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

        for (int i = 0; i < periodLength; i++) {
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
            measuredKinematics.getSpecificForceTriad(triad);
            triad.getMeasurementX(aX);
            triad.getMeasurementY(aY);
            triad.getMeasurementZ(aZ);

            assertTrue(detector.processAccelerationTriad(aX, aY, aZ));

            oldNedFrame.copyFrom(newNedFrame);
            oldEcefFrame.copyFrom(newEcefFrame);
            oldRoll = newRoll;
            oldPitch = newPitch;
            oldYaw = newYaw;
            oldEcefX = oldEcefFrame.getX();
            oldEcefY = oldEcefFrame.getY();
            oldEcefZ = oldEcefFrame.getZ();
        }

        assertEquals(mStaticIntervalDetected, 1);
        assertEquals(mDynamicIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                StaticIntervalDetector.Status.DYNAMIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + 2 * periodLength);

        // check that when switching to dynamic period, estimated average
        // specific force from last static period is approximately equal to the
        // true value
        assertEquals(lastStaticTriad.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(lastStaticTriad.getValueX(),
                detector.getAvgSpecificForceX(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueY(),
                detector.getAvgSpecificForceY(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueZ(),
                detector.getAvgSpecificForceZ(), ABSOLUTE_ERROR);
        assertTrue(lastStaticTriad.getMeasurementX().equals(
                detector.getAvgSpecificForceXAsAcceleration(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementY().equals(
                detector.getAvgSpecificForceYAsAcceleration(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementZ().equals(
                detector.getAvgSpecificForceZAsAcceleration(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.equals(detector.getAvgSpecificForce(),
                ABSOLUTE_ERROR));


        // keep adding static samples for twice the window size to last
        // true kinematics
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);
            triad.getMeasurementX(aX);
            triad.getMeasurementY(aY);
            triad.getMeasurementZ(aZ);

            assertTrue(detector.processAccelerationTriad(aX, aY, aZ));
        }

        assertEquals(mStaticIntervalDetected, 2);
        assertEquals(mDynamicIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                StaticIntervalDetector.Status.STATIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + 3 * periodLength);

        // reset
        detector.reset();

        assertEquals(mReset, 1);
        assertEquals(detector.getStatus(), StaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        a1 = detector.getBaseNoiseLevelAsAcceleration();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsAcceleration(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        a1 = detector.getThresholdAsAcceleration();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getThresholdAsAcceleration(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getProcessedSamples(), 0);
    }

    @Test
    public void testProcessAccelerationTriadWithSuccessfulInitializationStaticAndDynamicPeriodAndReset3()
            throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
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

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

        reset();
        assertEquals(mInitializationStarted, 0);
        assertEquals(mInitializationCompleted, 0);
        assertEquals(mError, 0);
        assertEquals(mStaticIntervalDetected, 0);
        assertEquals(mDynamicIntervalDetected, 0);
        assertEquals(mReset, 0);

        final StaticIntervalDetector detector =
                new StaticIntervalDetector(this);

        assertEquals(detector.getStatus(), StaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        Acceleration a1 = detector.getBaseNoiseLevelAsAcceleration();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsAcceleration(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        a1 = detector.getThresholdAsAcceleration();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getThresholdAsAcceleration(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getProcessedSamples(), 0);

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        final BodyKinematics measuredKinematics = new BodyKinematics();
        final AccelerationTriad triad = new AccelerationTriad();
        for (int i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.processAccelerationTriad(
                    triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(mInitializationStarted, 1);
        assertEquals(mInitializationCompleted, 1);
        assertEquals(detector.getStatus(),
                StaticIntervalDetector.Status.INITIALIZATION_COMPLETED);
        assertTrue(detector.getBaseNoiseLevel() > 0.0);
        a1 = detector.getBaseNoiseLevelAsAcceleration();
        assertEquals(a1.getValue().doubleValue(), detector.getBaseNoiseLevel(),
                0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsAcceleration(a2);
        assertEquals(a1, a2);
        assertTrue(detector.getThreshold() > 0.0);
        assertEquals(detector.getThreshold(),
                detector.getBaseNoiseLevel() * detector.getThresholdFactor(),
                0.0);
        a1 = detector.getThresholdAsAcceleration();
        assertEquals(a1.getValue().doubleValue(), detector.getThreshold(),
                0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getThresholdAsAcceleration(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getProcessedSamples(), initialStaticSamples);

        // keep adding static samples for twice the window size
        int periodLength = 2 * detector.getWindowSize();
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.processAccelerationTriad(
                    triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(mStaticIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                StaticIntervalDetector.Status.STATIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + periodLength);

        final AccelerationTriad lastStaticTriad =
                trueKinematics.getSpecificForceTriad();

        // add dynamic samples for twice the window size
        final double deltaX = randomizer.nextDouble(
                MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final double deltaY = randomizer.nextDouble(
                MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final double deltaZ = randomizer.nextDouble(
                MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);

        final double deltaRoll = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaPitch = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaYaw = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final double ecefX = ecefFrame.getX();
        final double ecefY = ecefFrame.getY();
        final double ecefZ = ecefFrame.getZ();

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

        for (int i = 0; i < periodLength; i++) {
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
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.processAccelerationTriad(
                    triad.getValueX(), triad.getValueY(), triad.getValueZ()));

            oldNedFrame.copyFrom(newNedFrame);
            oldEcefFrame.copyFrom(newEcefFrame);
            oldRoll = newRoll;
            oldPitch = newPitch;
            oldYaw = newYaw;
            oldEcefX = oldEcefFrame.getX();
            oldEcefY = oldEcefFrame.getY();
            oldEcefZ = oldEcefFrame.getZ();
        }

        assertEquals(mStaticIntervalDetected, 1);
        assertEquals(mDynamicIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                StaticIntervalDetector.Status.DYNAMIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + 2 * periodLength);

        // check that when switching to dynamic period, estimated average
        // specific force from last static period is approximately equal to the
        // true value
        assertEquals(lastStaticTriad.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(lastStaticTriad.getValueX(),
                detector.getAvgSpecificForceX(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueY(),
                detector.getAvgSpecificForceY(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueZ(),
                detector.getAvgSpecificForceZ(), ABSOLUTE_ERROR);
        assertTrue(lastStaticTriad.getMeasurementX().equals(
                detector.getAvgSpecificForceXAsAcceleration(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementY().equals(
                detector.getAvgSpecificForceYAsAcceleration(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementZ().equals(
                detector.getAvgSpecificForceZAsAcceleration(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.equals(detector.getAvgSpecificForce(),
                ABSOLUTE_ERROR));


        // keep adding static samples for twice the window size to last
        // true kinematics
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.processAccelerationTriad(
                    triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(mStaticIntervalDetected, 2);
        assertEquals(mDynamicIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                StaticIntervalDetector.Status.STATIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + 3 * periodLength);

        // reset
        detector.reset();

        assertEquals(mReset, 1);
        assertEquals(detector.getStatus(), StaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        a1 = detector.getBaseNoiseLevelAsAcceleration();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsAcceleration(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        a1 = detector.getThresholdAsAcceleration();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getThresholdAsAcceleration(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getProcessedSamples(), 0);
    }

    @Test
    public void testProcessAccelerationTriadWithExcessiveOverallNoiseDuringInitialization()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
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

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

        reset();
        assertEquals(mInitializationStarted, 0);
        assertEquals(mInitializationCompleted, 0);
        assertEquals(mError, 0);
        assertEquals(mStaticIntervalDetected, 0);
        assertEquals(mDynamicIntervalDetected, 0);
        assertEquals(mReset, 0);

        final StaticIntervalDetector detector =
                new StaticIntervalDetector(this);
        detector.setBaseNoiseLevelAbsoluteThreshold(Double.MIN_VALUE);

        assertEquals(detector.getStatus(), StaticIntervalDetector.Status.IDLE);

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        final BodyKinematics measuredKinematics = new BodyKinematics();
        final AccelerationTriad triad = new AccelerationTriad();
        for (int i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.processAccelerationTriad(triad));

            if (mError != 0) {
                break;
            }
        }

        assertEquals(mInitializationStarted, 1);
        assertEquals(mError, 1);
        assertEquals(detector.getStatus(), StaticIntervalDetector.Status.FAILED);
        assertTrue(mErrorAccumulatedNoiseLevel > 0.0);
        assertTrue(mErrorInstantaneousNoiseLevel > 0.0);
        assertEquals(mErrorReason,
                StaticIntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED);

        // attempting to process another triad after failure, is ignored
        assertFalse(detector.processAccelerationTriad(triad));

        // if we reset detector, we can process new samples
        detector.reset();

        assertTrue(detector.processAccelerationTriad(triad));
    }

    @Test
    public void testProcessAccelerationTriadWithSuddenMotionDuringInitialization()
            throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
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

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

        reset();
        assertEquals(mInitializationStarted, 0);
        assertEquals(mInitializationCompleted, 0);
        assertEquals(mError, 0);
        assertEquals(mStaticIntervalDetected, 0);
        assertEquals(mDynamicIntervalDetected, 0);
        assertEquals(mReset, 0);

        final StaticIntervalDetector detector =
                new StaticIntervalDetector(this);

        assertEquals(detector.getStatus(), StaticIntervalDetector.Status.IDLE);

        final int initialStaticSamples = detector.getInitialStaticSamples();
        int periodLength = 2 * detector.getWindowSize();

        assertTrue(initialStaticSamples > 2 * periodLength);
        int halfInitialStaticSamples = initialStaticSamples / 2;

        // add some sample while keeping accelerometer body static
        final BodyKinematics measuredKinematics = new BodyKinematics();
        final AccelerationTriad triad = new AccelerationTriad();
        for (int i = 0; i < halfInitialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.processAccelerationTriad(triad));
        }

        assertEquals(mInitializationStarted, 1);
        assertEquals(detector.getStatus(),
                StaticIntervalDetector.Status.INITIALIZING);

        // then add samples with motion
        final double deltaX = randomizer.nextDouble(
                MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final double deltaY = randomizer.nextDouble(
                MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);
        final double deltaZ = randomizer.nextDouble(
                MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS);

        final double deltaRoll = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaPitch = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaYaw = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final double ecefX = ecefFrame.getX();
        final double ecefY = ecefFrame.getY();
        final double ecefZ = ecefFrame.getZ();

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

        for (int i = 0; i < periodLength; i++) {
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
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.processAccelerationTriad(triad));

            oldNedFrame.copyFrom(newNedFrame);
            oldEcefFrame.copyFrom(newEcefFrame);
            oldRoll = newRoll;
            oldPitch = newPitch;
            oldYaw = newYaw;
            oldEcefX = oldEcefFrame.getX();
            oldEcefY = oldEcefFrame.getY();
            oldEcefZ = oldEcefFrame.getZ();

            if (mError != 0) {
                break;
            }
        }

        assertEquals(mInitializationStarted, 1);
        assertEquals(mError, 1);
        assertEquals(detector.getStatus(), StaticIntervalDetector.Status.FAILED);
        assertTrue(mErrorAccumulatedNoiseLevel > 0.0);
        assertTrue(mErrorInstantaneousNoiseLevel > 0.0);
        assertEquals(mErrorReason,
                StaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED);

        // attempting to process another triad after failure, is ignored
        assertFalse(detector.processAccelerationTriad(triad));

        // if we reset detector, we can process new samples
        detector.reset();

        assertTrue(detector.processAccelerationTriad(triad));
    }

    @Override
    public void onInitializationStarted(final StaticIntervalDetector detected) {
        mInitializationStarted++;
        checkLocked(detected);
        assertEquals(detected.getStatus(),
                StaticIntervalDetector.Status.INITIALIZING);
    }

    @Override
    public void onInitializationCompleted(final StaticIntervalDetector detected,
                                          final double baseNoiseLevel) {
        mInitializationCompleted++;
        checkLocked(detected);
        assertEquals(detected.getStatus(),
                StaticIntervalDetector.Status.INITIALIZATION_COMPLETED);
    }

    @Override
    public void onError(final StaticIntervalDetector detector,
                        final double accumulatedNoiseLevel,
                        final double instantaneousNoiseLevel,
                        final StaticIntervalDetector.ErrorReason reason) {
        mError++;
        mErrorAccumulatedNoiseLevel = accumulatedNoiseLevel;
        mErrorInstantaneousNoiseLevel = instantaneousNoiseLevel;
        mErrorReason = reason;
        checkLocked(detector);
    }

    @Override
    public void onStaticIntervalDetected(final StaticIntervalDetector detector) {
        mStaticIntervalDetected++;
        checkLocked(detector);
        assertEquals(detector.getStatus(),
                StaticIntervalDetector.Status.STATIC_INTERVAL);
    }

    @Override
    public void onDynamicIntervalDetected(final StaticIntervalDetector detector,
                                          final double avgSpecificForceX,
                                          final double avgSpecificForceY,
                                          final double avgSpecificForceZ) {
        mDynamicIntervalDetected++;
        checkLocked(detector);
        assertEquals(detector.getStatus(),
                StaticIntervalDetector.Status.DYNAMIC_INTERVAL);
        assertEquals(avgSpecificForceX, detector.getAvgSpecificForceX(), 0.0);
        assertEquals(avgSpecificForceY, detector.getAvgSpecificForceY(), 0.0);
        assertEquals(avgSpecificForceZ, detector.getAvgSpecificForceZ(), 0.0);

        final Acceleration ax1 = detector.getAvgSpecificForceXAsAcceleration();
        assertEquals(ax1.getValue().doubleValue(), avgSpecificForceX, 0.0);
        assertEquals(ax1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration ax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAvgSpecificForceXAsAcceleration(ax2);
        assertEquals(ax1, ax2);

        final Acceleration ay1 = detector.getAvgSpecificForceYAsAcceleration();
        assertEquals(ay1.getValue().doubleValue(), avgSpecificForceY, 0.0);
        assertEquals(ay1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration ay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAvgSpecificForceYAsAcceleration(ay2);
        assertEquals(ay1, ay2);

        final Acceleration az1 = detector.getAvgSpecificForceZAsAcceleration();
        assertEquals(az1.getValue().doubleValue(), avgSpecificForceZ, 0.0);
        assertEquals(az1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration az2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAvgSpecificForceZAsAcceleration(az2);
        assertEquals(az1, az2);

        final AccelerationTriad triad1 = detector.getAvgSpecificForce();
        assertEquals(triad1.getValueX(), avgSpecificForceX, 0.0);
        assertEquals(triad1.getValueY(), avgSpecificForceY, 0.0);
        assertEquals(triad1.getValueZ(), avgSpecificForceZ, 0.0);
        assertEquals(triad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final AccelerationTriad triad2 = new AccelerationTriad();
        detector.getAvgSpecificForce(triad2);
        assertEquals(triad1, triad2);
    }

    @Override
    public void onReset(final StaticIntervalDetector detector) {
        mReset++;
        checkLocked(detector);
    }

    private void reset() {
        mInitializationStarted = 0;
        mInitializationCompleted = 0;
        mError = 0;
        mStaticIntervalDetected = 0;
        mDynamicIntervalDetected = 0;
        mReset = 0;
    }

    private void checkLocked(final StaticIntervalDetector detector) {
        assertTrue(detector.isRunning());
        try {
            detector.setWindowSize(0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            detector.setInitialStaticSamples(0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            detector.setThresholdFactor(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            detector.setInstantaneousNoiseLevelFactor(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            detector.setBaseNoiseLevelAbsoluteThreshold(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            detector.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        final AccelerationTriad triad = new AccelerationTriad();
        try {
            detector.processAccelerationTriad(triad);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        final Acceleration a = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        try {
            detector.processAccelerationTriad(a, a, a);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            detector.processAccelerationTriad(0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            detector.reset();
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

}
