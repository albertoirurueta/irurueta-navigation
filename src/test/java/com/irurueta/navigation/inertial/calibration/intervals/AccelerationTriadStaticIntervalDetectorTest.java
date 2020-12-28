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
package com.irurueta.navigation.inertial.calibration.intervals;

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
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class AccelerationTriadStaticIntervalDetectorTest implements
        AccelerationTriadStaticIntervalDetectorListener {

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

    private static final double SMALL_ABSOLUTE_ERROR = 1e-3;

    private static final double VERY_SMALL_ABSOLUTE_ERROR = 1e-8;

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
        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector();

        assertEquals(detector.getWindowSize(),
                AccelerationTriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE);
        assertEquals(detector.getInitialStaticSamples(),
                AccelerationTriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES);
        assertEquals(detector.getThresholdFactor(),
                AccelerationTriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, 0.0);
        assertEquals(detector.getInstantaneousNoiseLevelFactor(),
                AccelerationTriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                0.0);
        assertEquals(detector.getBaseNoiseLevelAbsoluteThreshold(),
                AccelerationTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                0.0);
        assertNull(detector.getListener());
        assertEquals(detector.getTimeInterval(), TIME_INTERVAL_SECONDS, 0.0);
        final Time timeInterval1 = detector.getTimeIntervalAsTime();
        assertEquals(timeInterval1.getValue().doubleValue(),
                TIME_INTERVAL_SECONDS, 0.0);
        assertEquals(timeInterval1.getUnit(), TimeUnit.SECOND);
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        detector.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(detector.getStatus(), AccelerationTriadStaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        final Acceleration a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        final Acceleration a3 = detector.getThresholdAsMeasurement();
        assertEquals(a3.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a3.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a4 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getThresholdAsMeasurement(a4);
        assertEquals(a3, a4);
        assertFalse(detector.isRunning());
        assertEquals(detector.getProcessedSamples(), 0);

        assertEquals(detector.getAccumulatedAvgX(), 0.0, 0.0);
        final Acceleration a5 = detector.getAccumulatedAvgXAsMeasurement();
        assertEquals(a5.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a5.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a6 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgXAsMeasurement(a6);
        assertEquals(a5, a6);

        assertEquals(detector.getAccumulatedAvgY(), 0.0, 0.0);
        final Acceleration a7 = detector.getAccumulatedAvgYAsMeasurement();
        assertEquals(a7.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a7.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a8 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgYAsMeasurement(a8);
        assertEquals(a7, a8);

        assertEquals(detector.getAccumulatedAvgZ(), 0.0, 0.0);
        final Acceleration a9 = detector.getAccumulatedAvgZAsMeasurement();
        assertEquals(a9.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a9.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a10 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgZAsMeasurement(a10);
        assertEquals(a9, a10);

        final AccelerationTriad triad1 = detector.getAccumulatedAvgTriad();
        assertEquals(triad1.getValueX(), 0.0, 0.0);
        assertEquals(triad1.getValueY(), 0.0, 0.0);
        assertEquals(triad1.getValueZ(), 0.0, 0.0);
        assertEquals(triad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad triad2 = new AccelerationTriad();
        detector.getAccumulatedAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(detector.getAccumulatedStdX(), 0.0, 0.0);
        final Acceleration a11 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(a11.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a11.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a12 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdXAsMeasurement(a12);
        assertEquals(a11, a12);

        assertEquals(detector.getAccumulatedStdY(), 0.0, 0.0);
        final Acceleration a13 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(a13.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a13.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a14 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdYAsMeasurement(a14);
        assertEquals(a13, a14);

        assertEquals(detector.getAccumulatedStdZ(), 0.0, 0.0);
        final Acceleration a15 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(a15.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a15.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a16 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdZAsMeasurement(a16);
        assertEquals(a15, a16);

        final AccelerationTriad triad3 = detector.getAccumulatedStdTriad();
        assertEquals(triad3.getValueX(), 0.0, 0.0);
        assertEquals(triad3.getValueY(), 0.0, 0.0);
        assertEquals(triad3.getValueZ(), 0.0, 0.0);
        assertEquals(triad3.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad triad4 = new AccelerationTriad();
        detector.getAccumulatedStdTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(detector.getInstantaneousAvgX(), 0.0, 0.0);
        final Acceleration a17 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(a17.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a17.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a18 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgXAsMeasurement(a18);
        assertEquals(a17, a18);

        assertEquals(detector.getInstantaneousAvgY(), 0.0, 0.0);
        final Acceleration a19 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(a19.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a19.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a20 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgYAsMeasurement(a20);
        assertEquals(a19, a20);

        assertEquals(detector.getInstantaneousAvgZ(), 0.0, 0.0);
        final Acceleration a21 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(a21.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a21.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a22 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgZAsMeasurement(a22);
        assertEquals(a21, a22);

        final AccelerationTriad triad5 = detector.getInstantaneousAvgTriad();
        assertEquals(triad5.getValueX(), 0.0, 0.0);
        assertEquals(triad5.getValueY(), 0.0, 0.0);
        assertEquals(triad5.getValueZ(), 0.0, 0.0);
        assertEquals(triad5.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad triad6 = new AccelerationTriad();
        detector.getInstantaneousAvgTriad(triad6);
        assertEquals(triad5, triad6);

        assertEquals(detector.getInstantaneousStdX(), 0.0, 0.0);
        final Acceleration a23 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(a23.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a23.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a24 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdXAsMeasurement(a24);
        assertEquals(a23, a24);

        assertEquals(detector.getInstantaneousStdY(), 0.0, 0.0);
        final Acceleration a25 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(a25.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a25.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a26 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdYAsMeasurement(a26);
        assertEquals(a25, a26);

        assertEquals(detector.getInstantaneousStdZ(), 0.0, 0.0);
        final Acceleration a27 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(a27.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a27.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a28 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdZAsMeasurement(a28);
        assertEquals(a27, a28);

        final AccelerationTriad triad7 = detector.getInstantaneousStdTriad();
        assertEquals(triad7.getValueX(), 0.0, 0.0);
        assertEquals(triad7.getValueY(), 0.0, 0.0);
        assertEquals(triad7.getValueZ(), 0.0, 0.0);
        assertEquals(triad7.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad triad8 = new AccelerationTriad();
        detector.getInstantaneousStdTriad(triad8);
        assertEquals(triad7, triad8);
    }

    @Test
    public void testConstructor2() {
        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector(
                this);

        assertEquals(detector.getWindowSize(),
                AccelerationTriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE);
        assertEquals(detector.getInitialStaticSamples(),
                AccelerationTriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES);
        assertEquals(detector.getThresholdFactor(),
                AccelerationTriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, 0.0);
        assertEquals(detector.getInstantaneousNoiseLevelFactor(),
                AccelerationTriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                0.0);
        assertEquals(detector.getBaseNoiseLevelAbsoluteThreshold(),
                AccelerationTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                0.0);
        assertSame(detector.getListener(), this);
        final Time timeInterval1 = detector.getTimeIntervalAsTime();
        assertEquals(timeInterval1.getValue().doubleValue(),
                TIME_INTERVAL_SECONDS, 0.0);
        assertEquals(timeInterval1.getUnit(), TimeUnit.SECOND);
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        detector.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(detector.getStatus(), AccelerationTriadStaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        final Acceleration a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        final Acceleration a3 = detector.getThresholdAsMeasurement();
        assertEquals(a3.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a3.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a4 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getThresholdAsMeasurement(a4);
        assertEquals(a3, a4);
        assertFalse(detector.isRunning());
        assertEquals(detector.getProcessedSamples(), 0);

        assertEquals(detector.getAccumulatedAvgX(), 0.0, 0.0);
        final Acceleration a5 = detector.getAccumulatedAvgXAsMeasurement();
        assertEquals(a5.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a5.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a6 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgXAsMeasurement(a6);
        assertEquals(a5, a6);

        assertEquals(detector.getAccumulatedAvgY(), 0.0, 0.0);
        final Acceleration a7 = detector.getAccumulatedAvgYAsMeasurement();
        assertEquals(a7.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a7.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a8 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgYAsMeasurement(a8);
        assertEquals(a7, a8);

        assertEquals(detector.getAccumulatedAvgZ(), 0.0, 0.0);
        final Acceleration a9 = detector.getAccumulatedAvgZAsMeasurement();
        assertEquals(a9.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a9.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a10 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgZAsMeasurement(a10);
        assertEquals(a9, a10);

        final AccelerationTriad triad1 = detector.getAccumulatedAvgTriad();
        assertEquals(triad1.getValueX(), 0.0, 0.0);
        assertEquals(triad1.getValueY(), 0.0, 0.0);
        assertEquals(triad1.getValueZ(), 0.0, 0.0);
        assertEquals(triad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad triad2 = new AccelerationTriad();
        detector.getAccumulatedAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(detector.getAccumulatedStdX(), 0.0, 0.0);
        final Acceleration a11 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(a11.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a11.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a12 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdXAsMeasurement(a12);
        assertEquals(a11, a12);

        assertEquals(detector.getAccumulatedStdY(), 0.0, 0.0);
        final Acceleration a13 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(a13.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a13.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a14 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdYAsMeasurement(a14);
        assertEquals(a13, a14);

        assertEquals(detector.getAccumulatedStdZ(), 0.0, 0.0);
        final Acceleration a15 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(a15.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a15.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a16 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdZAsMeasurement(a16);
        assertEquals(a15, a16);

        final AccelerationTriad triad3 = detector.getAccumulatedStdTriad();
        assertEquals(triad3.getValueX(), 0.0, 0.0);
        assertEquals(triad3.getValueY(), 0.0, 0.0);
        assertEquals(triad3.getValueZ(), 0.0, 0.0);
        assertEquals(triad3.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad triad4 = new AccelerationTriad();
        detector.getAccumulatedStdTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(detector.getInstantaneousAvgX(), 0.0, 0.0);
        final Acceleration a17 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(a17.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a17.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a18 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgXAsMeasurement(a18);
        assertEquals(a17, a18);

        assertEquals(detector.getInstantaneousAvgY(), 0.0, 0.0);
        final Acceleration a19 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(a19.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a19.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a20 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgYAsMeasurement(a20);
        assertEquals(a19, a20);

        assertEquals(detector.getInstantaneousAvgZ(), 0.0, 0.0);
        final Acceleration a21 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(a21.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a21.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a22 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgZAsMeasurement(a22);
        assertEquals(a21, a22);

        final AccelerationTriad triad5 = detector.getInstantaneousAvgTriad();
        assertEquals(triad5.getValueX(), 0.0, 0.0);
        assertEquals(triad5.getValueY(), 0.0, 0.0);
        assertEquals(triad5.getValueZ(), 0.0, 0.0);
        assertEquals(triad5.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad triad6 = new AccelerationTriad();
        detector.getInstantaneousAvgTriad(triad6);
        assertEquals(triad5, triad6);

        assertEquals(detector.getInstantaneousStdX(), 0.0, 0.0);
        final Acceleration a23 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(a23.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a23.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a24 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdXAsMeasurement(a24);
        assertEquals(a23, a24);

        assertEquals(detector.getInstantaneousStdY(), 0.0, 0.0);
        final Acceleration a25 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(a25.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a25.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a26 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdYAsMeasurement(a26);
        assertEquals(a25, a26);

        assertEquals(detector.getInstantaneousStdZ(), 0.0, 0.0);
        final Acceleration a27 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(a27.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a27.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a28 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdZAsMeasurement(a28);
        assertEquals(a27, a28);

        final AccelerationTriad triad7 = detector.getInstantaneousStdTriad();
        assertEquals(triad7.getValueX(), 0.0, 0.0);
        assertEquals(triad7.getValueY(), 0.0, 0.0);
        assertEquals(triad7.getValueZ(), 0.0, 0.0);
        assertEquals(triad7.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad triad8 = new AccelerationTriad();
        detector.getInstantaneousStdTriad(triad8);
        assertEquals(triad7, triad8);
    }

    @Test
    public void testGetSetWindowSize() throws LockedException {
        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector();

        // check default value
        assertEquals(detector.getWindowSize(),
                AccelerationTriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE);

        // set new value
        detector.setWindowSize(3);

        // check
        assertEquals(detector.getWindowSize(), 3);

        // Force IllegalArgumentException
        try {
            detector.setWindowSize(1);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            detector.setWindowSize(2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetInitialStaticSamples() throws LockedException {
        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector();

        // check default value
        assertEquals(detector.getInitialStaticSamples(),
                AccelerationTriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES);

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
        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector();

        // check default value
        assertEquals(detector.getThresholdFactor(),
                AccelerationTriadStaticIntervalDetector.DEFAULT_THRESHOLD_FACTOR, 0.0);

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
        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector();

        // check default value
        assertEquals(detector.getInstantaneousNoiseLevelFactor(),
                AccelerationTriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
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
        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector();

        // check default value
        assertEquals(detector.getBaseNoiseLevelAbsoluteThreshold(),
                AccelerationTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
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

        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector();

        // check default value
        assertEquals(detector.getBaseNoiseLevelAbsoluteThreshold(),
                AccelerationTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                0.0);

        final Acceleration a1 = detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(a1.getValue().doubleValue(),
                AccelerationTriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // set new value
        final Acceleration a2 = new Acceleration(1.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.setBaseNoiseLevelAbsoluteThreshold(a2);

        // check
        final Acceleration a3 = detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        final Acceleration a4 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(a4);
        assertEquals(a2, a3);
        assertEquals(a2, a4);
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector();

        // check default value
        assertNull(detector.getListener());

        // set new value
        detector.setListener(this);

        // check
        assertSame(detector.getListener(), this);
    }

    @Test
    public void testGetSetTimeInterval1() throws LockedException {
        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector();

        // check default value
        assertEquals(detector.getTimeInterval(), TIME_INTERVAL_SECONDS, 0.0);

        // set new value
        final double timeInterval = 2 * TIME_INTERVAL_SECONDS;
        detector.setTimeInterval(timeInterval);

        // check
        assertEquals(detector.getTimeInterval(), timeInterval, 0.0);

        // Force IllegalArgumentException
        try {
            detector.setTimeInterval(-1.0);
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetTimeInterval2() throws LockedException {
        final AccelerationTriadStaticIntervalDetector detector = new AccelerationTriadStaticIntervalDetector();

        final Time timeInterval1 = detector.getTimeIntervalAsTime();
        assertEquals(timeInterval1.getValue().doubleValue(), TIME_INTERVAL_SECONDS, 0.0);
        assertEquals(timeInterval1.getUnit(), TimeUnit.SECOND);

        final Time timeInterval2 = new Time(2 * TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        detector.setTimeInterval(timeInterval2);

        final Time timeInterval3 = detector.getTimeIntervalAsTime();
        final Time timeInterval4 = new Time(1.0, TimeUnit.DAY);
        detector.getTimeIntervalAsTime(timeInterval4);

        assertEquals(timeInterval2, timeInterval3);
        assertEquals(timeInterval2, timeInterval4);
    }

    @Test
    public void testProcessWithSuccessfulInitializationStaticAndDynamicPeriodAndReset1()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final double accelNoiseStd = accelNoiseRootPSD /
                Math.sqrt(TIME_INTERVAL_SECONDS);

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

        final AccelerationTriad lastStaticTriad =
                trueKinematics.getSpecificForceTriad();

        reset();
        assertEquals(mInitializationStarted, 0);
        assertEquals(mInitializationCompleted, 0);
        assertEquals(mError, 0);
        assertEquals(mStaticIntervalDetected, 0);
        assertEquals(mDynamicIntervalDetected, 0);
        assertEquals(mReset, 0);

        final AccelerationTriadStaticIntervalDetector detector =
                new AccelerationTriadStaticIntervalDetector(this);

        assertEquals(detector.getStatus(), AccelerationTriadStaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        Acceleration a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getBaseNoiseLevelPsd(), 0.0, 0.0);
        assertEquals(detector.getBaseNoiseLevelRootPsd(), 0.0, 0.0);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        a1 = detector.getThresholdAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getThresholdAsMeasurement(a2);
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

            assertTrue(detector.process(triad));
        }

        assertEquals(mInitializationStarted, 1);
        assertEquals(mInitializationCompleted, 1);
        assertEquals(detector.getStatus(),
                AccelerationTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED);
        assertTrue(detector.getBaseNoiseLevel() > 0.0);
        a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), detector.getBaseNoiseLevel(),
                0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertTrue(detector.getBaseNoiseLevelPsd() > 0.0);
        assertTrue(detector.getBaseNoiseLevelRootPsd() > 0.0);
        assertEquals(detector.getBaseNoiseLevelRootPsd(),
                detector.getBaseNoiseLevel() * Math.sqrt(detector.getTimeInterval()),
                VERY_SMALL_ABSOLUTE_ERROR);
        assertEquals(detector.getBaseNoiseLevelPsd(),
                Math.pow(detector.getBaseNoiseLevelRootPsd(), 2.0),
                VERY_SMALL_ABSOLUTE_ERROR);
        assertTrue(detector.getThreshold() > 0.0);
        assertEquals(detector.getThreshold(),
                detector.getBaseNoiseLevel() * detector.getThresholdFactor(),
                0.0);
        a1 = detector.getThresholdAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), detector.getThreshold(),
                0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getThresholdAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getProcessedSamples(), initialStaticSamples);

        assertEquals(lastStaticTriad.getValueX(),
                detector.getAccumulatedAvgX(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueY(),
                detector.getAccumulatedAvgY(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueZ(),
                detector.getAccumulatedAvgZ(), ABSOLUTE_ERROR);
        assertTrue(lastStaticTriad.getMeasurementX().equals(
                detector.getAccumulatedAvgXAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementY().equals(
                detector.getAccumulatedAvgYAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementZ().equals(
                detector.getAccumulatedAvgZAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.equals(detector.getAccumulatedAvgTriad(),
                ABSOLUTE_ERROR));

        assertEquals(detector.getAccumulatedStdX(), accelNoiseStd, SMALL_ABSOLUTE_ERROR);
        assertEquals(detector.getAccumulatedStdY(), accelNoiseStd, SMALL_ABSOLUTE_ERROR);
        assertEquals(detector.getAccumulatedStdZ(), accelNoiseStd, SMALL_ABSOLUTE_ERROR);
        final Acceleration stdX1 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(stdX1.getValue().doubleValue(), accelNoiseStd,
                SMALL_ABSOLUTE_ERROR);
        assertEquals(stdX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration stdX2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        final Acceleration stdY1 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(stdY1.getValue().doubleValue(), accelNoiseStd,
                SMALL_ABSOLUTE_ERROR);
        assertEquals(stdY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration stdY2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        final Acceleration stdZ1 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(stdZ1.getValue().doubleValue(), accelNoiseStd,
                SMALL_ABSOLUTE_ERROR);
        assertEquals(stdZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration stdZ2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        // keep adding static samples for twice the window size
        int periodLength = 2 * detector.getWindowSize();
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad));
        }

        assertEquals(mStaticIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + periodLength);

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
                AccelerationTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + 2L * periodLength);

        // check that when switching to dynamic period, estimated average
        // specific force from last static period is approximately equal to the
        // true value
        assertEquals(lastStaticTriad.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(lastStaticTriad.getValueX(),
                detector.getAccumulatedAvgX(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueY(),
                detector.getAccumulatedAvgY(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueZ(),
                detector.getAccumulatedAvgZ(), ABSOLUTE_ERROR);
        assertTrue(lastStaticTriad.getMeasurementX().equals(
                detector.getAccumulatedAvgXAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementY().equals(
                detector.getAccumulatedAvgYAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementZ().equals(
                detector.getAccumulatedAvgZAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.equals(detector.getAccumulatedAvgTriad(),
                ABSOLUTE_ERROR));


        // keep adding static samples for twice the window size to last
        // true kinematics
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad));
        }

        assertEquals(mStaticIntervalDetected, 2);
        assertEquals(mDynamicIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + 3L * periodLength);

        // reset
        detector.reset();

        assertEquals(mReset, 1);
        assertEquals(detector.getStatus(), AccelerationTriadStaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getBaseNoiseLevelPsd(), 0.0, 0.0);
        assertEquals(detector.getBaseNoiseLevelRootPsd(), 0.0, 0.0);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        a1 = detector.getThresholdAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getThresholdAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getProcessedSamples(), 0);
    }

    @Test
    public void testProcessWithSuccessfulInitializationStaticAndDynamicPeriodAndReset2()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final double accelNoiseStd = accelNoiseRootPSD /
                Math.sqrt(TIME_INTERVAL_SECONDS);

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

        final AccelerationTriad lastStaticTriad =
                trueKinematics.getSpecificForceTriad();

        reset();
        assertEquals(mInitializationStarted, 0);
        assertEquals(mInitializationCompleted, 0);
        assertEquals(mError, 0);
        assertEquals(mStaticIntervalDetected, 0);
        assertEquals(mDynamicIntervalDetected, 0);
        assertEquals(mReset, 0);

        final AccelerationTriadStaticIntervalDetector detector =
                new AccelerationTriadStaticIntervalDetector(this);

        assertEquals(detector.getStatus(), AccelerationTriadStaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        Acceleration a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getBaseNoiseLevelPsd(), 0.0, 0.0);
        assertEquals(detector.getBaseNoiseLevelRootPsd(), 0.0, 0.0);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        a1 = detector.getThresholdAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getThresholdAsMeasurement(a2);
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

            assertTrue(detector.process(aX, aY, aZ));
        }

        assertEquals(mInitializationStarted, 1);
        assertEquals(mInitializationCompleted, 1);
        assertEquals(detector.getStatus(),
                AccelerationTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED);
        assertTrue(detector.getBaseNoiseLevel() > 0.0);
        a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), detector.getBaseNoiseLevel(),
                0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertTrue(detector.getBaseNoiseLevelPsd() > 0.0);
        assertTrue(detector.getBaseNoiseLevelRootPsd() > 0.0);
        assertEquals(detector.getBaseNoiseLevelRootPsd(),
                detector.getBaseNoiseLevel() * Math.sqrt(detector.getTimeInterval()),
                VERY_SMALL_ABSOLUTE_ERROR);
        assertEquals(detector.getBaseNoiseLevelPsd(),
                Math.pow(detector.getBaseNoiseLevelRootPsd(), 2.0),
                VERY_SMALL_ABSOLUTE_ERROR);
        assertTrue(detector.getThreshold() > 0.0);
        assertEquals(detector.getThreshold(),
                detector.getBaseNoiseLevel() * detector.getThresholdFactor(),
                0.0);
        a1 = detector.getThresholdAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), detector.getThreshold(),
                0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getThresholdAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getProcessedSamples(), initialStaticSamples);

        assertEquals(lastStaticTriad.getValueX(),
                detector.getAccumulatedAvgX(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueY(),
                detector.getAccumulatedAvgY(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueZ(),
                detector.getAccumulatedAvgZ(), ABSOLUTE_ERROR);
        assertTrue(lastStaticTriad.getMeasurementX().equals(
                detector.getAccumulatedAvgXAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementY().equals(
                detector.getAccumulatedAvgYAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementZ().equals(
                detector.getAccumulatedAvgZAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.equals(detector.getAccumulatedAvgTriad(),
                ABSOLUTE_ERROR));

        assertEquals(detector.getAccumulatedStdX(), accelNoiseStd, SMALL_ABSOLUTE_ERROR);
        assertEquals(detector.getAccumulatedStdY(), accelNoiseStd, SMALL_ABSOLUTE_ERROR);
        assertEquals(detector.getAccumulatedStdZ(), accelNoiseStd, SMALL_ABSOLUTE_ERROR);
        final Acceleration stdX1 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(stdX1.getValue().doubleValue(), accelNoiseStd,
                SMALL_ABSOLUTE_ERROR);
        assertEquals(stdX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration stdX2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        final Acceleration stdY1 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(stdY1.getValue().doubleValue(), accelNoiseStd,
                SMALL_ABSOLUTE_ERROR);
        assertEquals(stdY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration stdY2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        final Acceleration stdZ1 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(stdZ1.getValue().doubleValue(), accelNoiseStd,
                SMALL_ABSOLUTE_ERROR);
        assertEquals(stdZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration stdZ2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        // keep adding static samples for twice the window size
        int periodLength = 2 * detector.getWindowSize();
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);
            triad.getMeasurementX(aX);
            triad.getMeasurementY(aY);
            triad.getMeasurementZ(aZ);

            assertTrue(detector.process(aX, aY, aZ));
        }

        assertEquals(mStaticIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + periodLength);

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

            assertTrue(detector.process(aX, aY, aZ));

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
                AccelerationTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + 2L * periodLength);

        // check that when switching to dynamic period, estimated average
        // specific force from last static period is approximately equal to the
        // true value
        assertEquals(lastStaticTriad.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(lastStaticTriad.getValueX(),
                detector.getAccumulatedAvgX(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueY(),
                detector.getAccumulatedAvgY(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueZ(),
                detector.getAccumulatedAvgZ(), ABSOLUTE_ERROR);
        assertTrue(lastStaticTriad.getMeasurementX().equals(
                detector.getAccumulatedAvgXAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementY().equals(
                detector.getAccumulatedAvgYAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementZ().equals(
                detector.getAccumulatedAvgZAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.equals(detector.getAccumulatedAvgTriad(),
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

            assertTrue(detector.process(aX, aY, aZ));
        }

        assertEquals(mStaticIntervalDetected, 2);
        assertEquals(mDynamicIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + 3L * periodLength);

        // reset
        detector.reset();

        assertEquals(mReset, 1);
        assertEquals(detector.getStatus(), AccelerationTriadStaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getBaseNoiseLevelPsd(), 0.0, 0.0);
        assertEquals(detector.getBaseNoiseLevelRootPsd(), 0.0, 0.0);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        a1 = detector.getThresholdAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getThresholdAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getProcessedSamples(), 0);
    }

    @Test
    public void testProcessWithSuccessfulInitializationStaticAndDynamicPeriodAndReset3()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final double accelNoiseStd = accelNoiseRootPSD /
                Math.sqrt(TIME_INTERVAL_SECONDS);

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

        final AccelerationTriad lastStaticTriad =
                trueKinematics.getSpecificForceTriad();

        reset();
        assertEquals(mInitializationStarted, 0);
        assertEquals(mInitializationCompleted, 0);
        assertEquals(mError, 0);
        assertEquals(mStaticIntervalDetected, 0);
        assertEquals(mDynamicIntervalDetected, 0);
        assertEquals(mReset, 0);

        final AccelerationTriadStaticIntervalDetector detector =
                new AccelerationTriadStaticIntervalDetector(this);

        assertEquals(detector.getStatus(), AccelerationTriadStaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        Acceleration a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getBaseNoiseLevelPsd(), 0.0, 0.0);
        assertEquals(detector.getBaseNoiseLevelRootPsd(), 0.0, 0.0);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        a1 = detector.getThresholdAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getThresholdAsMeasurement(a2);
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

            assertTrue(detector.process(
                    triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(mInitializationStarted, 1);
        assertEquals(mInitializationCompleted, 1);
        assertEquals(detector.getStatus(),
                AccelerationTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED);
        assertTrue(detector.getBaseNoiseLevel() > 0.0);
        a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), detector.getBaseNoiseLevel(),
                0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertTrue(detector.getBaseNoiseLevelPsd() > 0.0);
        assertTrue(detector.getBaseNoiseLevelRootPsd() > 0.0);
        assertEquals(detector.getBaseNoiseLevelRootPsd(),
                detector.getBaseNoiseLevel() * Math.sqrt(detector.getTimeInterval()),
                VERY_SMALL_ABSOLUTE_ERROR);
        assertEquals(detector.getBaseNoiseLevelPsd(),
                Math.pow(detector.getBaseNoiseLevelRootPsd(), 2.0),
                VERY_SMALL_ABSOLUTE_ERROR);
        assertTrue(detector.getThreshold() > 0.0);
        assertEquals(detector.getThreshold(),
                detector.getBaseNoiseLevel() * detector.getThresholdFactor(),
                0.0);
        a1 = detector.getThresholdAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), detector.getThreshold(),
                0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getThresholdAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getProcessedSamples(), initialStaticSamples);

        assertEquals(lastStaticTriad.getValueX(),
                detector.getAccumulatedAvgX(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueY(),
                detector.getAccumulatedAvgY(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueZ(),
                detector.getAccumulatedAvgZ(), ABSOLUTE_ERROR);
        assertTrue(lastStaticTriad.getMeasurementX().equals(
                detector.getAccumulatedAvgXAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementY().equals(
                detector.getAccumulatedAvgYAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementZ().equals(
                detector.getAccumulatedAvgZAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.equals(detector.getAccumulatedAvgTriad(),
                ABSOLUTE_ERROR));

        assertEquals(detector.getAccumulatedStdX(), accelNoiseStd, SMALL_ABSOLUTE_ERROR);
        assertEquals(detector.getAccumulatedStdY(), accelNoiseStd, SMALL_ABSOLUTE_ERROR);
        assertEquals(detector.getAccumulatedStdZ(), accelNoiseStd, SMALL_ABSOLUTE_ERROR);
        final Acceleration stdX1 = detector.getAccumulatedStdXAsMeasurement();
        assertEquals(stdX1.getValue().doubleValue(), accelNoiseStd,
                SMALL_ABSOLUTE_ERROR);
        assertEquals(stdX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration stdX2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdXAsMeasurement(stdX2);
        assertEquals(stdX1, stdX2);
        final Acceleration stdY1 = detector.getAccumulatedStdYAsMeasurement();
        assertEquals(stdY1.getValue().doubleValue(), accelNoiseStd,
                SMALL_ABSOLUTE_ERROR);
        assertEquals(stdY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration stdY2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdYAsMeasurement(stdY2);
        assertEquals(stdY1, stdY2);
        final Acceleration stdZ1 = detector.getAccumulatedStdZAsMeasurement();
        assertEquals(stdZ1.getValue().doubleValue(), accelNoiseStd,
                SMALL_ABSOLUTE_ERROR);
        assertEquals(stdZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration stdZ2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedStdZAsMeasurement(stdZ2);
        assertEquals(stdZ1, stdZ2);

        // keep adding static samples for twice the window size
        int periodLength = 2 * detector.getWindowSize();
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(
                    triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(mStaticIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + periodLength);

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
                AccelerationTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + 2L * periodLength);

        // check that when switching to dynamic period, estimated average
        // specific force from last static period is approximately equal to the
        // true value
        assertEquals(lastStaticTriad.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertEquals(lastStaticTriad.getValueX(),
                detector.getAccumulatedAvgX(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueY(),
                detector.getAccumulatedAvgY(), ABSOLUTE_ERROR);
        assertEquals(lastStaticTriad.getValueZ(),
                detector.getAccumulatedAvgZ(), ABSOLUTE_ERROR);
        assertTrue(lastStaticTriad.getMeasurementX().equals(
                detector.getAccumulatedAvgXAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementY().equals(
                detector.getAccumulatedAvgYAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.getMeasurementZ().equals(
                detector.getAccumulatedAvgZAsMeasurement(), ABSOLUTE_ERROR));
        assertTrue(lastStaticTriad.equals(detector.getAccumulatedAvgTriad(),
                ABSOLUTE_ERROR));


        // keep adding static samples for twice the window size to last
        // true kinematics
        for (int i = 0; i < periodLength; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(
                    triad.getValueX(), triad.getValueY(), triad.getValueZ()));
        }

        assertEquals(mStaticIntervalDetected, 2);
        assertEquals(mDynamicIntervalDetected, 1);
        assertEquals(detector.getStatus(),
                AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL);
        assertEquals(detector.getProcessedSamples(),
                initialStaticSamples + 3L * periodLength);

        // reset
        detector.reset();

        assertEquals(mReset, 1);
        assertEquals(detector.getStatus(), AccelerationTriadStaticIntervalDetector.Status.IDLE);
        assertEquals(detector.getBaseNoiseLevel(), 0.0, 0.0);
        a1 = detector.getBaseNoiseLevelAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getBaseNoiseLevelAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getBaseNoiseLevelPsd(), 0.0, 0.0);
        assertEquals(detector.getBaseNoiseLevelRootPsd(), 0.0, 0.0);
        assertEquals(detector.getThreshold(), 0.0, 0.0);
        a1 = detector.getThresholdAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        detector.getThresholdAsMeasurement(a2);
        assertEquals(a1, a2);
        assertEquals(detector.getProcessedSamples(), 0);
    }

    @Test
    public void testProcessWithExcessiveOverallNoiseDuringInitialization() throws WrongSizeException,
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

        final AccelerationTriadStaticIntervalDetector detector =
                new AccelerationTriadStaticIntervalDetector(this);
        detector.setBaseNoiseLevelAbsoluteThreshold(Double.MIN_VALUE);

        assertEquals(detector.getStatus(), AccelerationTriadStaticIntervalDetector.Status.IDLE);

        final int initialStaticSamples = detector.getInitialStaticSamples();

        // generate enough measurements to complete initialization while keeping
        // accelerometer static
        final BodyKinematics measuredKinematics = new BodyKinematics();
        final AccelerationTriad triad = new AccelerationTriad();
        for (int i = 0; i < initialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad));

            if (mError != 0) {
                break;
            }
        }

        assertEquals(mInitializationStarted, 1);
        assertEquals(mError, 1);
        assertEquals(detector.getStatus(), AccelerationTriadStaticIntervalDetector.Status.FAILED);
        assertTrue(mErrorAccumulatedNoiseLevel > 0.0);
        assertTrue(mErrorInstantaneousNoiseLevel > 0.0);
        assertEquals(mErrorReason,
                AccelerationTriadStaticIntervalDetector.ErrorReason.OVERALL_EXCESSIVE_MOVEMENT_DETECTED);

        // attempting to process another triad after failure, is ignored
        assertFalse(detector.process(triad));

        // if we reset detector, we can process new samples
        detector.reset();

        assertTrue(detector.process(triad));
    }

    @Test
    public void testProcessWithSuddenMotionDuringInitialization() throws WrongSizeException,
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

        final AccelerationTriadStaticIntervalDetector detector =
                new AccelerationTriadStaticIntervalDetector(this);

        assertEquals(detector.getStatus(), AccelerationTriadStaticIntervalDetector.Status.IDLE);

        final int initialStaticSamples = detector.getInitialStaticSamples();
        int periodLength = 2 * detector.getWindowSize();

        assertTrue(initialStaticSamples > 2 * periodLength);
        int halfInitialStaticSamples = initialStaticSamples / 2;

        // add some samples while keeping accelerometer body static
        final BodyKinematics measuredKinematics = new BodyKinematics();
        final AccelerationTriad triad = new AccelerationTriad();
        for (int i = 0; i < halfInitialStaticSamples; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);
            measuredKinematics.getSpecificForceTriad(triad);

            assertTrue(detector.process(triad));
        }

        assertEquals(mInitializationStarted, 1);
        assertEquals(detector.getStatus(),
                AccelerationTriadStaticIntervalDetector.Status.INITIALIZING);

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
        assertEquals(detector.getStatus(), AccelerationTriadStaticIntervalDetector.Status.FAILED);
        assertTrue(mErrorAccumulatedNoiseLevel > 0.0);
        assertTrue(mErrorInstantaneousNoiseLevel > 0.0);
        assertEquals(mErrorReason,
                AccelerationTriadStaticIntervalDetector.ErrorReason.SUDDEN_EXCESSIVE_MOVEMENT_DETECTED);

        // attempting to process another triad after failure, is ignored
        assertFalse(detector.process(triad));

        // if we reset detector, we can process new samples
        detector.reset();

        assertTrue(detector.process(triad));
    }

    @Override
    public void onInitializationStarted(final AccelerationTriadStaticIntervalDetector detector) {
        mInitializationStarted++;
        checkLocked(detector);
        assertEquals(detector.getStatus(),
                AccelerationTriadStaticIntervalDetector.Status.INITIALIZING);
    }

    @Override
    public void onInitializationCompleted(final AccelerationTriadStaticIntervalDetector detector,
                                          final double baseNoiseLevel) {
        mInitializationCompleted++;
        checkLocked(detector);
        assertEquals(detector.getStatus(),
                AccelerationTriadStaticIntervalDetector.Status.INITIALIZATION_COMPLETED);
    }

    @Override
    public void onError(final AccelerationTriadStaticIntervalDetector detector,
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
    public void onStaticIntervalDetected(final AccelerationTriadStaticIntervalDetector detector,
                                         final double instantaneousAvgX,
                                         final double instantaneousAvgY,
                                         final double instantaneousAvgZ,
                                         final double instantaneousStdX,
                                         final double instantaneousStdY,
                                         final double instantaneousStdZ) {
        mStaticIntervalDetected++;
        checkLocked(detector);
        assertEquals(detector.getStatus(),
                AccelerationTriadStaticIntervalDetector.Status.STATIC_INTERVAL);

        assertEquals(detector.getInstantaneousAvgX(), instantaneousAvgX, 0.0);
        final Acceleration a1 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), instantaneousAvgX, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgXAsMeasurement(a2);
        assertEquals(a1, a2);

        assertEquals(detector.getInstantaneousAvgY(), instantaneousAvgY, 0.0);
        final Acceleration a3 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(a3.getValue().doubleValue(), instantaneousAvgY, 0.0);
        assertEquals(a3.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a4 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgYAsMeasurement(a4);
        assertEquals(a3, a4);

        assertEquals(detector.getInstantaneousAvgZ(), instantaneousAvgZ, 0.0);
        final Acceleration a5 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(a5.getValue().doubleValue(), instantaneousAvgZ, 0.0);
        assertEquals(a5.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a6 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgZAsMeasurement(a6);
        assertEquals(a5, a6);

        final AccelerationTriad avgTriad1 = detector.getInstantaneousAvgTriad();
        assertEquals(avgTriad1.getValueX(), instantaneousAvgX, 0.0);
        assertEquals(avgTriad1.getValueY(), instantaneousAvgY, 0.0);
        assertEquals(avgTriad1.getValueZ(), instantaneousAvgZ, 0.0);
        assertEquals(avgTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad avgTriad2 = new AccelerationTriad();
        detector.getInstantaneousAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        assertEquals(detector.getInstantaneousStdX(), instantaneousStdX, 0.0);
        final Acceleration a7 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(a7.getValue().doubleValue(), instantaneousStdX, 0.0);
        assertEquals(a7.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a8 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdXAsMeasurement(a8);
        assertEquals(a7, a8);

        assertEquals(detector.getInstantaneousStdY(), instantaneousStdY, 0.0);
        final Acceleration a9 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(a9.getValue().doubleValue(), instantaneousStdY, 0.0);
        assertEquals(a9.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a10 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdYAsMeasurement(a10);
        assertEquals(a9, a10);

        assertEquals(detector.getInstantaneousStdZ(), instantaneousStdZ, 0.0);
        final Acceleration a11 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(a11.getValue().doubleValue(), instantaneousStdZ, 0.0);
        assertEquals(a11.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a12 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdZAsMeasurement(a12);
        assertEquals(a11, a12);

        final AccelerationTriad stdTriad1 = detector.getInstantaneousStdTriad();
        assertTrue(stdTriad1.getNorm() < detector.getThreshold());
        assertEquals(stdTriad1.getValueX(), instantaneousStdX, 0.0);
        assertEquals(stdTriad1.getValueY(), instantaneousStdY, 0.0);
        assertEquals(stdTriad1.getValueZ(), instantaneousStdZ, 0.0);
        assertEquals(stdTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad stdTriad2 = new AccelerationTriad();
        detector.getInstantaneousStdTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
    }

    @Override
    public void onDynamicIntervalDetected(final AccelerationTriadStaticIntervalDetector detector,
                                          final double instantaneousAvgX,
                                          final double instantaneousAvgY,
                                          final double instantaneousAvgZ,
                                          final double instantaneousStdX,
                                          final double instantaneousStdY,
                                          final double instantaneousStdZ,
                                          final double accumulatedAvgX,
                                          final double accumulatedAvgY,
                                          final double accumulatedAvgZ,
                                          final double accumulatedStdX,
                                          final double accumulatedStdY,
                                          final double accumulatedStdZ) {
        mDynamicIntervalDetected++;
        checkLocked(detector);
        assertEquals(detector.getStatus(),
                AccelerationTriadStaticIntervalDetector.Status.DYNAMIC_INTERVAL);
        assertEquals(accumulatedAvgX, detector.getAccumulatedAvgX(), 0.0);
        assertEquals(accumulatedAvgY, detector.getAccumulatedAvgY(), 0.0);
        assertEquals(accumulatedAvgZ, detector.getAccumulatedAvgZ(), 0.0);

        final Acceleration ax1 = detector.getAccumulatedAvgXAsMeasurement();
        assertEquals(ax1.getValue().doubleValue(), accumulatedAvgX, 0.0);
        assertEquals(ax1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration ax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgXAsMeasurement(ax2);
        assertEquals(ax1, ax2);

        final Acceleration ay1 = detector.getAccumulatedAvgYAsMeasurement();
        assertEquals(ay1.getValue().doubleValue(), accumulatedAvgY, 0.0);
        assertEquals(ay1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration ay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgYAsMeasurement(ay2);
        assertEquals(ay1, ay2);

        final Acceleration az1 = detector.getAccumulatedAvgZAsMeasurement();
        assertEquals(az1.getValue().doubleValue(), accumulatedAvgZ, 0.0);
        assertEquals(az1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration az2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getAccumulatedAvgZAsMeasurement(az2);
        assertEquals(az1, az2);

        final AccelerationTriad triad1 = detector.getAccumulatedAvgTriad();
        assertEquals(triad1.getValueX(), accumulatedAvgX, 0.0);
        assertEquals(triad1.getValueY(), accumulatedAvgY, 0.0);
        assertEquals(triad1.getValueZ(), accumulatedAvgZ, 0.0);
        assertEquals(triad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final AccelerationTriad triad2 = new AccelerationTriad();
        detector.getAccumulatedAvgTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(detector.getInstantaneousAvgX(), instantaneousAvgX, 0.0);
        final Acceleration a1 = detector.getInstantaneousAvgXAsMeasurement();
        assertEquals(a1.getValue().doubleValue(), instantaneousAvgX, 0.0);
        assertEquals(a1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgXAsMeasurement(a2);
        assertEquals(a1, a2);

        assertEquals(detector.getInstantaneousAvgY(), instantaneousAvgY, 0.0);
        final Acceleration a3 = detector.getInstantaneousAvgYAsMeasurement();
        assertEquals(a3.getValue().doubleValue(), instantaneousAvgY, 0.0);
        assertEquals(a3.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a4 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgYAsMeasurement(a4);
        assertEquals(a3, a4);

        assertEquals(detector.getInstantaneousAvgZ(), instantaneousAvgZ, 0.0);
        final Acceleration a5 = detector.getInstantaneousAvgZAsMeasurement();
        assertEquals(a5.getValue().doubleValue(), instantaneousAvgZ, 0.0);
        assertEquals(a5.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a6 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousAvgZAsMeasurement(a6);
        assertEquals(a5, a6);

        final AccelerationTriad avgTriad1 = detector.getInstantaneousAvgTriad();
        assertEquals(avgTriad1.getValueX(), instantaneousAvgX, 0.0);
        assertEquals(avgTriad1.getValueY(), instantaneousAvgY, 0.0);
        assertEquals(avgTriad1.getValueZ(), instantaneousAvgZ, 0.0);
        assertEquals(avgTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad avgTriad2 = new AccelerationTriad();
        detector.getInstantaneousAvgTriad(avgTriad2);
        assertEquals(avgTriad1, avgTriad2);

        assertEquals(detector.getInstantaneousStdX(), instantaneousStdX, 0.0);
        final Acceleration a7 = detector.getInstantaneousStdXAsMeasurement();
        assertEquals(a7.getValue().doubleValue(), instantaneousStdX, 0.0);
        assertEquals(a7.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a8 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdXAsMeasurement(a8);
        assertEquals(a7, a8);

        assertEquals(detector.getInstantaneousStdY(), instantaneousStdY, 0.0);
        final Acceleration a9 = detector.getInstantaneousStdYAsMeasurement();
        assertEquals(a9.getValue().doubleValue(), instantaneousStdY, 0.0);
        assertEquals(a9.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a10 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdYAsMeasurement(a10);
        assertEquals(a9, a10);

        assertEquals(detector.getInstantaneousStdZ(), instantaneousStdZ, 0.0);
        final Acceleration a11 = detector.getInstantaneousStdZAsMeasurement();
        assertEquals(a11.getValue().doubleValue(), instantaneousStdZ, 0.0);
        assertEquals(a11.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration a12 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        detector.getInstantaneousStdZAsMeasurement(a12);
        assertEquals(a11, a12);

        final AccelerationTriad stdTriad1 = detector.getInstantaneousStdTriad();
        assertTrue(stdTriad1.getNorm() >= detector.getThreshold());
        assertEquals(stdTriad1.getValueX(), instantaneousStdX, 0.0);
        assertEquals(stdTriad1.getValueY(), instantaneousStdY, 0.0);
        assertEquals(stdTriad1.getValueZ(), instantaneousStdZ, 0.0);
        assertEquals(stdTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad stdTriad2 = new AccelerationTriad();
        detector.getInstantaneousStdTriad(stdTriad2);
        assertEquals(stdTriad1, stdTriad2);
    }

    @Override
    public void onReset(final AccelerationTriadStaticIntervalDetector detector) {
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

    private void checkLocked(final AccelerationTriadStaticIntervalDetector detector) {
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
        try {
            detector.setTimeInterval(0.0);
        } catch (final LockedException ignore) {
        }
        final Time timeInterval = new Time(1.0, TimeUnit.DAY);
        try {
            detector.setTimeInterval(timeInterval);
        } catch (final LockedException ignore) {
        }
        final AccelerationTriad triad = new AccelerationTriad();
        try {
            detector.process(triad);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        final Acceleration a = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        try {
            detector.process(a, a, a);
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
