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
import com.irurueta.navigation.geodesic.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityUnit;
import org.junit.Test;

import java.io.IOException;
import java.util.Calendar;
import java.util.Date;
import java.util.Random;

import static org.junit.Assert.*;

public class MagneticFluxDensityTriadStaticIntervalDetectorTest implements
        MagneticFluxDensityTriadStaticIntervalDetectorListener {

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

    private static final double MIN_DELTA_POS_METERS = -0.01;
    private static final double MAX_DELTA_POS_METERS = 0.01;
    private static final double MIN_DELTA_ANGLE_DEGREES = -2.0;
    private static final double MAX_DELTA_ANGLE_DEGREES = 2.0;

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

    private int mInitializationStarted;

    private int mInitializationCompleted;

    private int mError;

    private int mStaticIntervalDetected;

    private int mDynamicIntervalDetected;

    private int mReset;

    private double mErrorAccumulatedNoiseLevel;

    private double mErrorInstantaneousNoiseLevel;

    private AccelerationTriadStaticIntervalDetector.ErrorReason mErrorReason;

    @Test
    public void testConstructor1() {
        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector();

        assertEquals(detector.getWindowSize(),
                MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE);
        assertEquals(detector.getInitialStaticSamples(),
                MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES);
        assertEquals(detector.getThresholdFactor(),
                MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, 0.0);
        assertEquals(detector.getInstantaneousNoiseLevelFactor(),
                MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                0.0);
        assertEquals(detector.getBaseNoiseLevelAbsoluteThreshold(),
                MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                0.0);
        assertNull(detector.getListener());
        assertEquals(detector.getStatus(), MagneticFluxDensityTriadStaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        final MagneticFluxDensity b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        final MagneticFluxDensity b3 = detector.getThresholdAsMeasurement();
        assertEquals(b3.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b3.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b4 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
        detector.getThresholdAsMeasurement(b4);
        assertEquals(b3, b4);
        assertFalse(detector.isRunning());
        assertEquals(detector.getProcessedSamples(), 0);
    }

    @Test
    public void testConstructor2() {
        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector(this);

        assertEquals(detector.getWindowSize(),
                MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE);
        assertEquals(detector.getInitialStaticSamples(),
                MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES);
        assertEquals(detector.getThresholdFactor(),
                MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, 0.0);
        assertEquals(detector.getInstantaneousNoiseLevelFactor(),
                MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                0.0);
        assertEquals(detector.getBaseNoiseLevelAbsoluteThreshold(),
                MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                0.0);
        assertSame(detector.getListener(), this);
        assertEquals(detector.getStatus(), MagneticFluxDensityTriadStaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        final MagneticFluxDensity b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        final MagneticFluxDensity b3 = detector.getThresholdAsMeasurement();
        assertEquals(b3.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b3.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b4 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
        detector.getThresholdAsMeasurement(b4);
        assertEquals(b3, b4);
        assertFalse(detector.isRunning());
        assertEquals(detector.getProcessedSamples(), 0);
    }

    @Test
    public void testGetSetWindowSize() throws LockedException {
        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector();

        // check default value
        assertEquals(detector.getWindowSize(),
                MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE);

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
        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector();

        // check default value
        assertEquals(detector.getInitialStaticSamples(),
                MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES);

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
        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector();

        // check default value
        assertEquals(detector.getThresholdFactor(),
                MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, 0.0);

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
        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector();

        // check default value
        assertEquals(detector.getInstantaneousNoiseLevelFactor(),
                MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
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
        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector();

        // check default value
        assertEquals(detector.getBaseNoiseLevelAbsoluteThreshold(),
                MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
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
    public void testGetSetBaseNoiseLevelAbsoluteThresholdAsMeasurement()
            throws LockedException {

        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector();

        // check default value
        assertEquals(detector.getBaseNoiseLevelAbsoluteThreshold(),
                MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                0.0);

        final MagneticFluxDensity b1 = detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(b1.getValue().doubleValue(),
                MagneticFluxDensityTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);

        // set new value
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        detector.setBaseNoiseLevelAbsoluteThreshold(b2);

        // check
        final MagneticFluxDensity b3 = detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        final MagneticFluxDensity b4 = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.TESLA);
        detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(b4);
        assertEquals(b2, b3);
        assertEquals(b2, b4);
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector();

        // check default value
        assertNull(detector.getListener());

        // set new value
        detector.setListener(this);

        // check
        assertSame(detector.getListener(), this);
    }

    @Test
    public void testProcessWithSuccessfulInitializationStaticAndDynamicPeriodAndReset1()
            throws InvalidSourceAndDestinationFrameTypeException, IOException, LockedException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, MAGNETOMETER_NOISE_STD);

        final NEDPosition nedPosition = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));

        final CoordinateTransformation nedC = cnb.inverseAndReturnNew();

        final double roll = nedC.getRollEulerAngle();
        final double pitch = nedC.getPitchEulerAngle();
        final double yaw = nedC.getYawEulerAngle();

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth body magnetic flux density at provided
        // timestamp, position, and orientation
        MagneticFluxDensityTriad trueB = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                null, timestamp, nedPosition, cnb);

        reset();
        assertEquals(mInitializationStarted, 0);
        assertEquals(mInitializationCompleted, 0);
        assertEquals(mError, 0);
        assertEquals(mStaticIntervalDetected, 0);
        assertEquals(mDynamicIntervalDetected, 0);
        assertEquals(mReset, 0);

        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector(this);

        assertEquals(detector.getStatus(), MagneticFluxDensityTriadStaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        MagneticFluxDensity b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        b1 = detector.getThresholdAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        detector.getThresholdAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(detector.getProcessedSamples(), 0);

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        MagneticFluxDensityTriad triad;
        for (int i = 0; i < initialStaticSamples; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, nedPosition, cnb);

            assertTrue(detector.process(triad));
        }

        assertEquals(mInitializationStarted, 1);
        assertEquals(mInitializationCompleted, 1);
        assertEquals(detector.getStatus(),
                MagneticFluxDensityTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED);
        assertTrue(detector.getBaseNoiseLevel() > 0.0);
        b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), detector.getBaseNoiseLevel(),
                0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertTrue(detector.getThreshold() > 0.0);
        assertEquals(detector.getThreshold(),
                detector.getBaseNoiseLevel() * detector.getThresholdFactor(),
                0.0);
        b1 = detector.getThresholdAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), detector.getThreshold(),
                0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        detector.getThresholdAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(detector.getProcessedSamples(), initialStaticSamples);

        // keep adding static samples for twice the window size
        int periodLength = 2 * detector.getWindowSize();
        for (int i = 0; i < periodLength; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, nedPosition, cnb);

            assertTrue(detector.process(triad));
        }

        assertEquals(mStaticIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                MagneticFluxDensityTriadStaticIntervalDetector.Status.STATIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + periodLength);

        final MagneticFluxDensityTriad lastStaticTriad = new MagneticFluxDensityTriad(trueB);

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
            newNedC.inverse(cnb);

            final NEDPosition newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final double newEcefX = oldEcefX + deltaX;
            final double newEcefY = oldEcefY + deltaY;
            final double newEcefZ = oldEcefZ + deltaZ;

            newEcefFrame.setCoordinates(newEcefX, newEcefY, newEcefZ);

            ECEFtoNEDFrameConverter.convertECEFtoNED(newEcefFrame, newNedFrame);

            // update true magnetic flux density using new position and rotation
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, nedPosition, cnb);

            assertTrue(detector.process(triad));

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
                MagneticFluxDensityTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + 2 * periodLength);

        // check that when switching to dynamic period, estimated average
        // magnetic flux density from last static period is approximately equal to the
        // true value
        assertEquals(lastStaticTriad.getUnit(),
                MagneticFluxDensityUnit.TESLA);
        assertEquals(lastStaticTriad.getValueX(),
                detector.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueY(),
                detector.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueZ(),
                detector.getAvgZ(), ABSOLUTE_ERROR);
        assertTrue(lastStaticTriad.getMeasurementX().equals(
                detector.getAvgXAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementY().equals(
                detector.getAvgYAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementZ().equals(
                detector.getAvgZAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.equals(detector.getAvgTriad(),
                ABSOLUTE_ERROR));

        // keep adding static samples for twice the window size to last
        // true magnetic flux density
        for (int i = 0; i < periodLength; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, nedPosition, cnb);

            assertTrue(detector.process(triad));
        }

        assertEquals(mStaticIntervalDetected, 2);
        assertEquals(mDynamicIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                MagneticFluxDensityTriadStaticIntervalDetector.Status.STATIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + 3 * periodLength);

        // reset
        detector.reset();

        assertEquals(mReset, 1);
        assertEquals(detector.getStatus(), MagneticFluxDensityTriadStaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        b1 = detector.getThresholdAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        detector.getThresholdAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(detector.getProcessedSamples(), 0);
    }

    @Test
    public void testProcessWithSuccessfulInitializationStaticAndDynamicPeriodAndReset2()
            throws InvalidSourceAndDestinationFrameTypeException, IOException, LockedException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, MAGNETOMETER_NOISE_STD);

        final NEDPosition nedPosition = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));

        final CoordinateTransformation nedC = cnb.inverseAndReturnNew();

        final double roll = nedC.getRollEulerAngle();
        final double pitch = nedC.getPitchEulerAngle();
        final double yaw = nedC.getYawEulerAngle();

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth body magnetic flux density at provided
        // timestamp, position, and orientation
        MagneticFluxDensityTriad trueB = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                null, timestamp, nedPosition, cnb);

        reset();
        assertEquals(mInitializationStarted, 0);
        assertEquals(mInitializationCompleted, 0);
        assertEquals(mError, 0);
        assertEquals(mStaticIntervalDetected, 0);
        assertEquals(mDynamicIntervalDetected, 0);
        assertEquals(mReset, 0);

        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector(this);

        assertEquals(detector.getStatus(), MagneticFluxDensityTriadStaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        MagneticFluxDensity b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        b1 = detector.getThresholdAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        detector.getThresholdAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(detector.getProcessedSamples(), 0);

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        final MagneticFluxDensity bX = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity bY = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity bZ = new MagneticFluxDensity(
                0.0, MagneticFluxDensityUnit.TESLA);
        MagneticFluxDensityTriad triad;
        for (int i = 0; i < initialStaticSamples; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, nedPosition, cnb);
            triad.getMeasurementX(bX);
            triad.getMeasurementY(bY);
            triad.getMeasurementZ(bZ);

            assertTrue(detector.process(bX, bY, bZ));
        }

        assertEquals(mInitializationStarted, 1);
        assertEquals(mInitializationCompleted, 1);
        assertEquals(detector.getStatus(),
                MagneticFluxDensityTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED);
        assertTrue(detector.getBaseNoiseLevel() > 0.0);
        b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), detector.getBaseNoiseLevel(),
                0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertTrue(detector.getThreshold() > 0.0);
        assertEquals(detector.getThreshold(),
                detector.getBaseNoiseLevel() * detector.getThresholdFactor(),
                0.0);
        b1 = detector.getThresholdAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), detector.getThreshold(),
                0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        detector.getThresholdAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(detector.getProcessedSamples(), initialStaticSamples);

        // keep adding static samples for twice the window size
        int periodLength = 2 * detector.getWindowSize();
        for (int i = 0; i < periodLength; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, nedPosition, cnb);
            triad.getMeasurementX(bX);
            triad.getMeasurementY(bY);
            triad.getMeasurementZ(bZ);

            assertTrue(detector.process(bX, bY, bZ));
        }

        assertEquals(mStaticIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                MagneticFluxDensityTriadStaticIntervalDetector.Status.STATIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + periodLength);

        final MagneticFluxDensityTriad lastStaticTriad = new MagneticFluxDensityTriad(trueB);

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
            newNedC.inverse(cnb);

            final NEDPosition newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final double newEcefX = oldEcefX + deltaX;
            final double newEcefY = oldEcefY + deltaY;
            final double newEcefZ = oldEcefZ + deltaZ;

            newEcefFrame.setCoordinates(newEcefX, newEcefY, newEcefZ);

            ECEFtoNEDFrameConverter.convertECEFtoNED(newEcefFrame, newNedFrame);

            // update true magnetic flux density using new position and rotation
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, nedPosition, cnb);
            triad.getMeasurementX(bX);
            triad.getMeasurementY(bY);
            triad.getMeasurementZ(bZ);

            assertTrue(detector.process(bX, bY, bZ));

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
                MagneticFluxDensityTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + 2 * periodLength);

        // check that when switching to dynamic period, estimated average
        // magnetic flux density from last static period is approximately equal to the
        // true value
        assertEquals(lastStaticTriad.getUnit(),
                MagneticFluxDensityUnit.TESLA);
        assertEquals(lastStaticTriad.getValueX(),
                detector.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueY(),
                detector.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueZ(),
                detector.getAvgZ(), ABSOLUTE_ERROR);
        assertTrue(lastStaticTriad.getMeasurementX().equals(
                detector.getAvgXAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementY().equals(
                detector.getAvgYAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementZ().equals(
                detector.getAvgZAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.equals(detector.getAvgTriad(),
                ABSOLUTE_ERROR));

        // keep adding static samples for twice the window size to last
        // true magnetic flux density
        for (int i = 0; i < periodLength; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, nedPosition, cnb);
            triad.getMeasurementX(bX);
            triad.getMeasurementY(bY);
            triad.getMeasurementZ(bZ);

            assertTrue(detector.process(bX, bY, bZ));
        }

        assertEquals(mStaticIntervalDetected, 2);
        assertEquals(mDynamicIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                MagneticFluxDensityTriadStaticIntervalDetector.Status.STATIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + 3 * periodLength);

        // reset
        detector.reset();

        assertEquals(mReset, 1);
        assertEquals(detector.getStatus(), MagneticFluxDensityTriadStaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        b1 = detector.getThresholdAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        detector.getThresholdAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(detector.getProcessedSamples(), 0);
    }

    @Test
    public void testProcessWithSuccessfulInitializationStaticAndDynamicPeriodAndReset3()
            throws InvalidSourceAndDestinationFrameTypeException, IOException, LockedException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, MAGNETOMETER_NOISE_STD);

        final NEDPosition nedPosition = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));

        final CoordinateTransformation nedC = cnb.inverseAndReturnNew();

        final double roll = nedC.getRollEulerAngle();
        final double pitch = nedC.getPitchEulerAngle();
        final double yaw = nedC.getYawEulerAngle();

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth body magnetic flux density at provided
        // timestamp, position, and orientation
        MagneticFluxDensityTriad trueB = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                null, timestamp, nedPosition, cnb);

        reset();
        assertEquals(mInitializationStarted, 0);
        assertEquals(mInitializationCompleted, 0);
        assertEquals(mError, 0);
        assertEquals(mStaticIntervalDetected, 0);
        assertEquals(mDynamicIntervalDetected, 0);
        assertEquals(mReset, 0);

        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector(this);

        assertEquals(detector.getStatus(), MagneticFluxDensityTriadStaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        MagneticFluxDensity b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        b1 = detector.getThresholdAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        detector.getThresholdAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(detector.getProcessedSamples(), 0);

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        MagneticFluxDensityTriad triad;
        for (int i = 0; i < initialStaticSamples; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, nedPosition, cnb);

            assertTrue(detector.process(
                    triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(mInitializationStarted, 1);
        assertEquals(mInitializationCompleted, 1);
        assertEquals(detector.getStatus(),
                MagneticFluxDensityTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED);
        assertTrue(detector.getBaseNoiseLevel() > 0.0);
        b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), detector.getBaseNoiseLevel(),
                0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertTrue(detector.getThreshold() > 0.0);
        assertEquals(detector.getThreshold(),
                detector.getBaseNoiseLevel() * detector.getThresholdFactor(),
                0.0);
        b1 = detector.getThresholdAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), detector.getThreshold(),
                0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        detector.getThresholdAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(detector.getProcessedSamples(), initialStaticSamples);

        // keep adding static samples for twice the window size
        int periodLength = 2 * detector.getWindowSize();
        for (int i = 0; i < periodLength; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, nedPosition, cnb);

            assertTrue(detector.process(
                    triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(mStaticIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                MagneticFluxDensityTriadStaticIntervalDetector.Status.STATIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + periodLength);

        final MagneticFluxDensityTriad lastStaticTriad = new MagneticFluxDensityTriad(trueB);

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
            newNedC.inverse(cnb);

            final NEDPosition newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final double newEcefX = oldEcefX + deltaX;
            final double newEcefY = oldEcefY + deltaY;
            final double newEcefZ = oldEcefZ + deltaZ;

            newEcefFrame.setCoordinates(newEcefX, newEcefY, newEcefZ);

            ECEFtoNEDFrameConverter.convertECEFtoNED(newEcefFrame, newNedFrame);

            // update true magnetic flux density using new position and rotation
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, nedPosition, cnb);

            assertTrue(detector.process(
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
                MagneticFluxDensityTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + 2 * periodLength);

        // check that when switching to dynamic period, estimated average
        // magnetic flux density from last static period is approximately equal to the
        // true value
        assertEquals(lastStaticTriad.getUnit(),
                MagneticFluxDensityUnit.TESLA);
        assertEquals(lastStaticTriad.getValueX(),
                detector.getAvgX(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueY(),
                detector.getAvgY(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueZ(),
                detector.getAvgZ(), ABSOLUTE_ERROR);
        assertTrue(lastStaticTriad.getMeasurementX().equals(
                detector.getAvgXAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementY().equals(
                detector.getAvgYAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementZ().equals(
                detector.getAvgZAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.equals(detector.getAvgTriad(),
                ABSOLUTE_ERROR));

        // keep adding static samples for twice the window size to last
        // true magnetic flux density
        for (int i = 0; i < periodLength; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, nedPosition, cnb);

            assertTrue(detector.process(
                    triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(mStaticIntervalDetected, 2);
        assertEquals(mDynamicIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                MagneticFluxDensityTriadStaticIntervalDetector.Status.STATIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + 3 * periodLength);

        // reset
        detector.reset();

        assertEquals(mReset, 1);
        assertEquals(detector.getStatus(), MagneticFluxDensityTriadStaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        b1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        detector.getBaseNoiseLevelAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        b1 = detector.getThresholdAsMeasurement();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        detector.getThresholdAsMeasurement(b2);
        assertEquals(b1, b2);
        assertEquals(detector.getProcessedSamples(), 0);
    }

    @Test
    public void testProcessWithExcessiveOverallNoiseDuringInitialization()
            throws IOException, LockedException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, MAGNETOMETER_NOISE_STD);

        final NEDPosition nedPosition = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));

        reset();
        assertEquals(mInitializationStarted, 0);
        assertEquals(mInitializationCompleted, 0);
        assertEquals(mError, 0);
        assertEquals(mStaticIntervalDetected, 0);
        assertEquals(mDynamicIntervalDetected, 0);
        assertEquals(mReset, 0);

        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector(this);
        detector.setBaseNoiseLevelAbsoluteThreshold(Double.MIN_VALUE);

        assertEquals(detector.getStatus(), MagneticFluxDensityTriadStaticIntervalDetector.Status.IDLE);

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();
        for (int i = 0; i < initialStaticSamples; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, nedPosition, cnb);

            assertTrue(detector.process(triad));

            if (mError != 0) {
                break;
            }
        }

        assertEquals(mInitializationStarted, 1);
        assertEquals(mError, 1);
        assertEquals(detector.getStatus(), MagneticFluxDensityTriadStaticIntervalDetector.Status.FAILED);
        assertTrue(mErrorAccumulatedNoiseLevel > 0.0);
        assertTrue(mErrorInstantaneousNoiseLevel > 0.0);
        assertEquals(mErrorReason,
                MagneticFluxDensityTriadStaticIntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED);

        // attempting to process another triad after failure, is ignored
        assertFalse(detector.process(triad));

        // if we reset detector, we can process new samples
        detector.reset();

        assertTrue(detector.process(triad));
    }

    @Test
    public void testProcessWithSuddenMotionDuringInitialization()
            throws InvalidSourceAndDestinationFrameTypeException, IOException, LockedException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, MAGNETOMETER_NOISE_STD);

        final NEDPosition nedPosition = createPosition(randomizer);
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        final Date timestamp = new Date(createTimestamp(randomizer));

        final CoordinateTransformation nedC = cnb.inverseAndReturnNew();

        final double roll = nedC.getRollEulerAngle();
        final double pitch = nedC.getPitchEulerAngle();
        final double yaw = nedC.getYawEulerAngle();

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        reset();
        assertEquals(mInitializationStarted, 0);
        assertEquals(mInitializationCompleted, 0);
        assertEquals(mError, 0);
        assertEquals(mStaticIntervalDetected, 0);
        assertEquals(mDynamicIntervalDetected, 0);
        assertEquals(mReset, 0);

        final MagneticFluxDensityTriadStaticIntervalDetector detector =
                new MagneticFluxDensityTriadStaticIntervalDetector(this);

        assertEquals(detector.getStatus(), MagneticFluxDensityTriadStaticIntervalDetector.Status.IDLE);

        final int initialStaticSamples = detector.getInitialStaticSamples();
        int periodLength = 2 * detector.getWindowSize();

        assertTrue(initialStaticSamples > 2 * periodLength);
        int halfInitialStaticSamples = initialStaticSamples / 2;

        // add some samples while keeping magnetometer body static
        MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();
        for (int i = 0; i < halfInitialStaticSamples; i++) {
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, nedPosition, cnb);

            assertTrue(detector.process(triad));
        }

        assertEquals(mInitializationStarted, 1);
        assertEquals(detector.getStatus(),
                MagneticFluxDensityTriadStaticIntervalDetector.Status.INITIALIZING);

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
            newNedC.inverse(cnb);

            final NEDPosition newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final double newEcefX = oldEcefX + deltaX;
            final double newEcefY = oldEcefY + deltaY;
            final double newEcefZ = oldEcefZ + deltaZ;

            newEcefFrame.setCoordinates(newEcefX, newEcefY, newEcefZ);

            ECEFtoNEDFrameConverter.convertECEFtoNED(newEcefFrame, newNedFrame);

            // update true magnetic flux density using new position and rotation
            triad = generateTriad(hardIron.getBuffer(), mm, wmmEstimator,
                    noiseRandomizer, timestamp, nedPosition, cnb);

            assertTrue(detector.process(triad));

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
        assertEquals(detector.getStatus(), MagneticFluxDensityTriadStaticIntervalDetector.Status.FAILED);
        assertTrue(mErrorAccumulatedNoiseLevel > 0.0);
        assertTrue(mErrorInstantaneousNoiseLevel > 0.0);
        assertEquals(mErrorReason,
                MagneticFluxDensityTriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED);

        // attempting to process another triad after failure, is ignored
        assertFalse(detector.process(triad));

        // if we reset detector, we can process new samples
        detector.reset();

        assertTrue(detector.process(triad));
    }

    @Override
    public void onInitializationStarted(final MagneticFluxDensityTriadStaticIntervalDetector detector) {
        mInitializationStarted++;
        checkLocked(detector);
        assertEquals(detector.getStatus(),
                MagneticFluxDensityTriadStaticIntervalDetector.Status.INITIALIZING);
    }

    @Override
    public void onInitializationCompleted(final MagneticFluxDensityTriadStaticIntervalDetector detector,
                                          final double baseNoiseLevel) {
        mInitializationCompleted++;
        checkLocked(detector);
        assertEquals(detector.getStatus(),
                MagneticFluxDensityTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED);
    }

    @Override
    public void onError(final MagneticFluxDensityTriadStaticIntervalDetector detector,
                        final double accumulatedNoiseLevel,
                        final double instantaneousNoiseLevel,
                        final TriadStaticIntervalDetector.ErrorReason reason) {
        mError++;
        mErrorAccumulatedNoiseLevel = accumulatedNoiseLevel;
        mErrorInstantaneousNoiseLevel = instantaneousNoiseLevel;
        mErrorReason = reason;
        checkLocked(detector);
    }

    @Override
    public void onStaticIntervalDetected(final MagneticFluxDensityTriadStaticIntervalDetector detector) {
        mStaticIntervalDetected++;
        checkLocked(detector);
        assertEquals(detector.getStatus(),
                MagneticFluxDensityTriadStaticIntervalDetector.Status.STATIC_INTERVAL);
    }

    @Override
    public void onDynamicIntervalDetected(final MagneticFluxDensityTriadStaticIntervalDetector detector,
                                          final double avgX, final double avgY, final double avgZ) {
        mDynamicIntervalDetected++;
        checkLocked(detector);
        assertEquals(detector.getStatus(),
                MagneticFluxDensityTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL);
        assertEquals(avgX, detector.getAvgX(), 0.0);
        assertEquals(avgY, detector.getAvgY(), 0.0);
        assertEquals(avgZ, detector.getAvgZ(), 0.0);

        final MagneticFluxDensity bx1 = detector.getAvgXAsMeasurement();
        assertEquals(bx1.getValue().doubleValue(), avgX, 0.0);
        assertEquals(bx1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity bx2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
        detector.getAvgXAsMeasurement(bx2);
        assertEquals(bx1, bx2);

        final MagneticFluxDensity by1 = detector.getAvgYAsMeasurement();
        assertEquals(by1.getValue().doubleValue(), avgY, 0.0);
        assertEquals(by1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity by2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
        detector.getAvgYAsMeasurement(by2);
        assertEquals(by1, by2);

        final MagneticFluxDensity bz1 = detector.getAvgZAsMeasurement();
        assertEquals(bz1.getValue().doubleValue(), avgZ, 0.0);
        assertEquals(bz1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity bz2 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
        detector.getAvgZAsMeasurement(bz2);
        assertEquals(bz1, bz2);

        final MagneticFluxDensityTriad triad1 = detector.getAvgTriad();
        assertEquals(triad1.getValueX(), avgX, 0.0);
        assertEquals(triad1.getValueY(), avgY, 0.0);
        assertEquals(triad1.getValueZ(), avgZ, 0.0);
        assertEquals(triad1.getUnit(), MagneticFluxDensityUnit.TESLA);

        final MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad();
        detector.getAvgTriad(triad2);
        assertEquals(triad1, triad2);
    }

    @Override
    public void onReset(final MagneticFluxDensityTriadStaticIntervalDetector detector) {
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

    private void checkLocked(final MagneticFluxDensityTriadStaticIntervalDetector detector) {
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
        final MagneticFluxDensityTriad triad = new MagneticFluxDensityTriad();
        try {
            detector.process(triad);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        final MagneticFluxDensity b = new MagneticFluxDensity(0.0,
                MagneticFluxDensityUnit.TESLA);
        try {
            detector.process(b, b, b);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            detector.process(0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            detector.reset();
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
