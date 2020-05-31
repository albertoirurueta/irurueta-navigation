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
package com.irurueta.navigation.inertial.calibration.accelerometer;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.FrameBodyKinematics;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibratorTest implements
        KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibratorListener {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREEs = 90.0;
    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;
    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final int SMALL_MEASUREMENT_NUMBER = 16;
    private static final int LARGE_MEASUREMENT_NUMBER = 100000;

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-5;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;

    private static final int TIMES = 100;

    private int mCalibrateStart;
    private int mCalibrateEnd;

    @Test
    public void testConstructor() throws WrongSizeException {
        // test empty constructor
        KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator();

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        Acceleration acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        double[] bias1 = new double[3];
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with listener
        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[3];
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with measurements
        final Collection<FrameBodyKinematics> measurements = Collections.emptyList();

        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                measurements);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[3];
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with measurements and listener
        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                measurements, this);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[3];
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with common axis used flag
        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                true);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[3];
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with common axis used flag and listener
        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                true, this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[3];
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with measurements and common axis flag
        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                measurements, true);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[3];
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with measurements, common axis flag and listener
        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                measurements, true, this);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(0.0,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[3];
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with bias
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                biasX, biasY, biasZ);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(biasX,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(biasY,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(biasZ,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[]{ biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with bias and listener
        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                biasX, biasY, biasZ, this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(biasX,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(biasY,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(biasZ,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[]{ biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with measurements and bias
        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                measurements, biasX, biasY, biasZ);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(biasX,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(biasY,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(biasZ,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[]{ biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with measurements, bias and listener
        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                measurements, biasX, biasY, biasZ, this);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(biasX,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(biasY,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(biasZ,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[]{ biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with bias and common axis flag
        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                biasX, biasY, biasZ, true);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(biasX,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(biasY,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(biasZ,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[]{ biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with bias, common axis flag and listener
        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                biasX, biasY, biasZ, true, this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(biasX,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(biasY,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(biasZ,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[]{ biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with measurements, bias and common axis flag
        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                measurements, biasX, biasY, biasZ, true);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(biasX,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(biasY,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(biasZ,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[]{ biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with measurements, bias, common axis flag and listener
        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                measurements, biasX, biasY, biasZ, true, this);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(biasX,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(biasY,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(biasZ,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[]{ biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with bias as acceleration
        final Acceleration biasXAcceleration = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration biasYAcceleration = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration biasZAcceleration = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                biasXAcceleration, biasYAcceleration, biasZAcceleration);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(biasX,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(biasY,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(biasZ,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[]{ biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with bias as acceleration and listener
        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                biasXAcceleration, biasYAcceleration, biasZAcceleration, this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(biasX,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(biasY,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(biasZ,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[]{ biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with measurements and bias as acceleration
        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                measurements, biasXAcceleration, biasYAcceleration, biasZAcceleration);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(biasX,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(biasY,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(biasZ,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[]{ biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with measurements, bias as acceleration and listener
        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                measurements, biasXAcceleration, biasYAcceleration, biasZAcceleration,
                this);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(biasX,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(biasY,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(biasZ,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[]{ biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with bias as acceleration and common axis used flag
        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                biasXAcceleration, biasYAcceleration, biasZAcceleration,
                true);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(biasX,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(biasY,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(biasZ,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[]{ biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with bias as acceleration and common axis used flag and
        // listener
        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                biasXAcceleration, biasYAcceleration, biasZAcceleration,
                true, this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(biasX,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(biasY,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(biasZ,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[]{ biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with measurements, bias as acceleration and common axis
        // flag
        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                measurements, biasXAcceleration, biasYAcceleration, biasZAcceleration,
                true);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(biasX,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(biasY,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(biasZ,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[]{ biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());


        // test constructor with measurements, bias as acceleration, common axis
        // flag and listener
        calibrator = new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(
                measurements, biasXAcceleration, biasYAcceleration, biasZAcceleration,
                true, this);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        assertEquals(calibrator.getBiasXAsAcceleration(),
                new Acceleration(biasX,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasYAsAcceleration(),
                new Acceleration(biasY,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(calibrator.getBiasZAsAcceleration(),
                new Acceleration(biasZ,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        acceleration = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration);
        assertEquals(acceleration, new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND));
        bias1 = new double[]{ biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getEstimatedMa());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
    }

    @Test
    public void testGetSetMeasurements() throws LockedException {
        final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getMeasurements());

        // set new value
        final Collection<FrameBodyKinematics> measurements = Collections.emptyList();
        calibrator.setMeasurements(measurements);

        // check
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testIsSetCommonAxisUsed() throws LockedException {
        final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testGetSetBiasX() throws LockedException {
        final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);

        // set new value
        final Random random = new Random();
        final double biasX = random.nextDouble();

        calibrator.setBiasX(biasX);

        // check
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
    }

    @Test
    public void testGetSetBiasY() throws LockedException {
        final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);

        // set new value
        final Random random = new Random();
        final double biasY = random.nextDouble();

        calibrator.setBiasY(biasY);

        // check
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
    }

    @Test
    public void testGetSetBiasZ() throws LockedException {
        final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);

        // set new value
        final Random random = new Random();
        final double biasZ = random.nextDouble();

        calibrator.setBiasZ(biasZ);

        // check
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
    }

    @Test
    public void testGetSetBiasXAsAcceleration() throws LockedException {
        final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator();

        // check default value
        final Acceleration biasX1 = calibrator.getBiasYAsAcceleration();

        assertEquals(biasX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(biasX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // set new value
        final Random random = new Random();
        final Acceleration biasX2 = new Acceleration(random.nextDouble(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setBiasX(biasX2);

        // check
        final Acceleration biasX3 = calibrator.getBiasXAsAcceleration();
        final Acceleration biasX4 = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(biasX4);

        assertEquals(biasX2, biasX3);
        assertEquals(biasX2, biasX4);
    }

    @Test
    public void testGetSetBiasYAsAcceleration() throws LockedException {
        final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator();

        // check default value
        final Acceleration biasY1 = calibrator.getBiasYAsAcceleration();

        assertEquals(biasY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(biasY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // set new value
        final Random random = new Random();
        final Acceleration biasY2 = new Acceleration(random.nextDouble(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setBiasY(biasY2);

        // check
        final Acceleration biasY3 = calibrator.getBiasYAsAcceleration();
        final Acceleration biasY4 = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(biasY4);

        assertEquals(biasY2, biasY3);
        assertEquals(biasY2, biasY4);
    }

    @Test
    public void testGetSetBiasZAsAcceleration() throws LockedException {
        final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator();

        // check default value
        final Acceleration biasZ1 = calibrator.getBiasZAsAcceleration();

        assertEquals(biasZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(biasZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // set new value
        final Random random = new Random();
        final Acceleration biasZ2 = new Acceleration(random.nextDouble(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setBiasZ(biasZ2);

        // check
        final Acceleration biasZ3 = calibrator.getBiasZAsAcceleration();
        final Acceleration biasZ4 = new Acceleration(0.0,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(biasZ4);

        assertEquals(biasZ2, biasZ3);
        assertEquals(biasZ2, biasZ4);
    }

    @Test
    public void testSetBiasCoordinates() throws LockedException {
        final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);

        // set new values
        final Random random = new Random();
        final double biasX = random.nextDouble();
        final double biasY = random.nextDouble();
        final double biasZ = random.nextDouble();

        calibrator.setBiasCoordinates(biasX, biasY, biasZ);

        // check
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
    }

    @Test
    public void testSetBiasCoordinatesAcceleration() throws LockedException {
        final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);

        // set new values
        final Random random = new Random();
        final double biasX = random.nextDouble();
        final double biasY = random.nextDouble();
        final double biasZ = random.nextDouble();

        final Acceleration bx = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration by = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration bz = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setBiasCoordinates(bx, by, bz);

        // check
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
    }

    @Test
    public void testGetSetBiasAsArray() throws LockedException {
        final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator();

        final double[] bias1 = calibrator.getBias();
        final double[] bias2 = new double[BodyKinematics.COMPONENTS];
        calibrator.getBias(bias2);

        assertArrayEquals(bias1, new double[BodyKinematics.COMPONENTS], 0.0);
        assertArrayEquals(bias1, bias2, 0.0);

        // set new values
        Matrix ba = generateBa();
        final double[] bias3 = ba.getBuffer();

        calibrator.setBias(bias3);

        // check
        final double[] bias4 = new double[BodyKinematics.COMPONENTS];
        calibrator.getBias(bias4);
        final double[] bias5 = calibrator.getBias();

        assertArrayEquals(bias3, bias4, 0.0);
        assertArrayEquals(bias3, bias5, 0.0);

        // Force IllegalArgumentException
        try {
            calibrator.getBias(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setBias(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetBiasAsMatrix() throws WrongSizeException, LockedException {
        final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator();

        final Matrix bias1 = calibrator.getBiasAsMatrix();
        final Matrix bias2 = new Matrix(BodyKinematics.COMPONENTS, 1);
        calibrator.getBiasAsMatrix(bias2);

        assertEquals(bias1.getElementAtIndex(0), 0.0, 0.0);
        assertEquals(bias1.getElementAtIndex(1), 0.0, 0.0);
        assertEquals(bias1.getElementAtIndex(2), 0.0, 0.0);
        assertEquals(bias1, bias2);

        // set new value
        final Matrix bias3 = generateBa();
        calibrator.setBias(bias3);

        // check
        final Matrix bias4 = calibrator.getBiasAsMatrix();
        final Matrix bias5 = new Matrix(BodyKinematics.COMPONENTS, 1);
        calibrator.getBiasAsMatrix(bias5);

        assertEquals(bias3, bias4);
        assertEquals(bias3, bias5);

        // Force IllegalArgumentException
        try {
            calibrator.getBiasAsMatrix(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.getBiasAsMatrix(new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setBias(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setBias(new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testCalibrateMultipleOrientationsForGeneralCaseWithMinimumMeasuresAndNoNoise()
            throws WrongSizeException, LockedException, NotReadyException,
            CalibrationException, InvalidSourceAndDestinationFrameTypeException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        // when using minimum number of measurements we must not add any noise so that
        // a solution is found. When adding more measurements, certain noise can be added
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

        final List<FrameBodyKinematics> measurements = new ArrayList<>();
        for (int i = 0; i < KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

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
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame,
                            ecefFrame);

            // apply known calibration parameters to distort ground-truth and generate a
            // measured kinematics sample
            final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                    .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random);

            final FrameBodyKinematics measurement = new FrameBodyKinematics(
                    measuredKinematics, ecefFrame, ecefFrame, TIME_INTERVAL_SECONDS);
            measurements.add(measurement);
        }

        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(measurements,
                        biasX, biasY, biasZ, false, this);

        // estimate
        reset();
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(mCalibrateStart, 0);
        assertEquals(mCalibrateEnd, 0);

        calibrator.calibrate();

        // check
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(mCalibrateStart, 1);
        assertEquals(mCalibrateEnd, 1);

        final Matrix estimatedMa = calibrator.getEstimatedMa();

        assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMa, calibrator);
    }

    @Test
    public void testCalibrateMultipleOrientationsForGeneralCaseWithNoiseLargeNumberOfMeasurements()
            throws WrongSizeException, LockedException, NotReadyException,
            CalibrationException, InvalidSourceAndDestinationFrameTypeException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMaGeneral();
            final Matrix mg = generateMg();
            final Matrix gg = generateGg();
            // when using a larger number of measurements noise can be added to obtain a
            // meaningful least squares solution
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                    gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final List<FrameBodyKinematics> measurements = new ArrayList<>();
            for (int i = 0; i < LARGE_MEASUREMENT_NUMBER; i++) {

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
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame,
                                ecefFrame);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                        .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random);

                final FrameBodyKinematics measurement = new FrameBodyKinematics(
                        measuredKinematics, ecefFrame, ecefFrame, TIME_INTERVAL_SECONDS);
                measurements.add(measurement);
            }

            final double biasX = ba.getElementAtIndex(0);
            final double biasY = ba.getElementAtIndex(1);
            final double biasZ = ba.getElementAtIndex(2);

            final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                    new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(measurements,
                            biasX, biasY, biasZ, false, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);

            final Matrix estimatedMa = calibrator.getEstimatedMa();

            if (!ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMa, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultipleOrientationsForGeneralCaseWithNoiseSmallNumberOfMeasurements()
            throws WrongSizeException, LockedException, NotReadyException,
            CalibrationException, InvalidSourceAndDestinationFrameTypeException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        // when using a larger number of measurements noise can be added to obtain a
        // meaningful least squares solution
        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

        final List<FrameBodyKinematics> measurements = new ArrayList<>();
        for (int i = 0; i < SMALL_MEASUREMENT_NUMBER; i++) {

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
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame,
                            ecefFrame);

            // apply known calibration parameters to distort ground-truth and generate a
            // measured kinematics sample
            final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                    .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random);

            final FrameBodyKinematics measurement = new FrameBodyKinematics(
                    measuredKinematics, ecefFrame, ecefFrame, TIME_INTERVAL_SECONDS);
            measurements.add(measurement);
        }

        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(measurements,
                        biasX, biasY, biasZ, false, this);

        // estimate
        reset();
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(mCalibrateStart, 0);
        assertEquals(mCalibrateEnd, 0);

        calibrator.calibrate();

        // check
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(mCalibrateStart, 1);
        assertEquals(mCalibrateEnd, 1);

        final Matrix estimatedMa = calibrator.getEstimatedMa();

        assertTrue(ma.equals(estimatedMa, VERY_LARGE_ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMa, calibrator);
    }

    @Test
    public void testCalibrateMultiplePositionsForGeneralCaseWithMinimumMeasuresAndNoNoise()
            throws WrongSizeException, LockedException, NotReadyException,
            CalibrationException, InvalidSourceAndDestinationFrameTypeException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMaGeneral();
            final Matrix mg = generateMg();
            final Matrix gg = generateGg();
            // when using minimum number of measurements we must not add any noise so that
            // a solution is found. When adding more measurements, certain noise can be added
            final double accelNoiseRootPSD = 0.0;
            final double gyroNoiseRootPSD = 0.0;
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                    gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double roll = 0.0;
            final double pitch = 0.0;
            final double yaw = 0.0;
            final CoordinateTransformation nedC = new CoordinateTransformation(
                    roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final List<FrameBodyKinematics> measurements = new ArrayList<>();
            for (int i = 0; i < KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

                final double latitude = Math.toRadians(
                        randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
                final double longitude = Math.toRadians(
                        randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
                final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
                final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

                final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
                final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame,
                                ecefFrame);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                        .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random);

                final FrameBodyKinematics measurement = new FrameBodyKinematics(
                        measuredKinematics, ecefFrame, ecefFrame, TIME_INTERVAL_SECONDS);
                measurements.add(measurement);
            }

            final double biasX = ba.getElementAtIndex(0);
            final double biasY = ba.getElementAtIndex(1);
            final double biasZ = ba.getElementAtIndex(2);

            final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                    new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(measurements,
                            biasX, biasY, biasZ, false, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);

            final Matrix estimatedMa = calibrator.getEstimatedMa();

            if (!ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMa, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultipleOrientationsForCommonAxisCaseWithMinimumMeasuresAndNoNoise()
            throws WrongSizeException, LockedException, NotReadyException,
            CalibrationException, InvalidSourceAndDestinationFrameTypeException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        // when using minimum number of measurements we must not add any noise so that
        // a solution is found. When adding more measurements, certain noise can be added
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

        final List<FrameBodyKinematics> measurements = new ArrayList<>();
        for (int i = 0; i < KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

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
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame,
                            ecefFrame);

            // apply known calibration parameters to distort ground-truth and generate a
            // measured kinematics sample
            final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                    .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random);

            final FrameBodyKinematics measurement = new FrameBodyKinematics(
                    measuredKinematics, ecefFrame, ecefFrame, TIME_INTERVAL_SECONDS);
            measurements.add(measurement);
        }

        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(measurements,
                        biasX, biasY, biasZ, true, this);

        // estimate
        reset();
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(mCalibrateStart, 0);
        assertEquals(mCalibrateEnd, 0);

        calibrator.calibrate();

        // check
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(mCalibrateStart, 1);
        assertEquals(mCalibrateEnd, 1);

        final Matrix estimatedMa = calibrator.getEstimatedMa();

        assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMa, calibrator);
    }

    @Test
    public void testCalibrateMultipleOrientationsForCommonAxisCaseWithNoiseLargeNumberOfMeasurements()
            throws WrongSizeException, LockedException, NotReadyException,
            CalibrationException, InvalidSourceAndDestinationFrameTypeException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMaCommonAxis();
            final Matrix mg = generateMg();
            final Matrix gg = generateGg();
            // when using a larger number of measurements noise can be added to obtain a
            // meaningful least squares solution
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                    gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final List<FrameBodyKinematics> measurements = new ArrayList<>();
            for (int i = 0; i < LARGE_MEASUREMENT_NUMBER; i++) {

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
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame,
                                ecefFrame);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                        .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random);

                final FrameBodyKinematics measurement = new FrameBodyKinematics(
                        measuredKinematics, ecefFrame, ecefFrame, TIME_INTERVAL_SECONDS);
                measurements.add(measurement);
            }

            final double biasX = ba.getElementAtIndex(0);
            final double biasY = ba.getElementAtIndex(1);
            final double biasZ = ba.getElementAtIndex(2);

            final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                    new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(measurements,
                            biasX, biasY, biasZ, true, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);

            final Matrix estimatedMa = calibrator.getEstimatedMa();

            if (!ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMa, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultipleOrientationsForCommonAxisCaseWithNoiseSmallNumberOfMeasurements()
            throws WrongSizeException, LockedException, NotReadyException,
            CalibrationException, InvalidSourceAndDestinationFrameTypeException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        // when using a larger number of measurements noise can be added to obtain a
        // meaningful least squares solution
        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

        final List<FrameBodyKinematics> measurements = new ArrayList<>();
        for (int i = 0; i < SMALL_MEASUREMENT_NUMBER; i++) {

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
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame,
                            ecefFrame);

            // apply known calibration parameters to distort ground-truth and generate a
            // measured kinematics sample
            final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                    .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random);

            final FrameBodyKinematics measurement = new FrameBodyKinematics(
                    measuredKinematics, ecefFrame, ecefFrame, TIME_INTERVAL_SECONDS);
            measurements.add(measurement);
        }

        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(measurements,
                        biasX, biasY, biasZ, false, this);

        // estimate
        reset();
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(mCalibrateStart, 0);
        assertEquals(mCalibrateEnd, 0);

        calibrator.calibrate();

        // check
        assertTrue(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(mCalibrateStart, 1);
        assertEquals(mCalibrateEnd, 1);

        final Matrix estimatedMa = calibrator.getEstimatedMa();

        assertTrue(ma.equals(estimatedMa, VERY_LARGE_ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMa, calibrator);
    }

    @Test
    public void testCalibrateMultiplePositionsForCommonAxisCaseWithMinimumMeasuresAndNoNoise()
            throws WrongSizeException, LockedException, NotReadyException,
            CalibrationException, InvalidSourceAndDestinationFrameTypeException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMaCommonAxis();
            final Matrix mg = generateMg();
            final Matrix gg = generateGg();
            // when using minimum number of measurements we must not add any noise so that
            // a solution is found. When adding more measurements, certain noise can be added
            final double accelNoiseRootPSD = 0.0;
            final double gyroNoiseRootPSD = 0.0;
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                    gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double roll = 0.0;
            final double pitch = 0.0;
            final double yaw = 0.0;
            final CoordinateTransformation nedC = new CoordinateTransformation(
                    roll, pitch, yaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);

            final List<FrameBodyKinematics> measurements = new ArrayList<>();
            for (int i = 0; i < KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

                final double latitude = Math.toRadians(
                        randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
                final double longitude = Math.toRadians(
                        randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
                final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
                final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

                final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
                final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefFrame,
                                ecefFrame);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                        .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random);

                final FrameBodyKinematics measurement = new FrameBodyKinematics(
                        measuredKinematics, ecefFrame, ecefFrame, TIME_INTERVAL_SECONDS);
                measurements.add(measurement);
            }

            final double biasX = ba.getElementAtIndex(0);
            final double biasY = ba.getElementAtIndex(1);
            final double biasZ = ba.getElementAtIndex(2);

            final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator =
                    new KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator(measurements,
                            biasX, biasY, biasZ, true, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);

            final Matrix estimatedMa = calibrator.getEstimatedMa();

            if (!ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMa, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        mCalibrateStart++;
    }

    @Override
    public void onCalibrateEnd(KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        mCalibrateEnd++;
    }

    private void reset() {
        mCalibrateStart = 0;
        mCalibrateEnd = 0;
    }

    private void checkLocked(
            final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
        try {
            calibrator.setMeasurements(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setCommonAxisUsed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setListener(this);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setBiasX(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setBiasY(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setBiasZ(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        final Acceleration acceleration = new Acceleration(
                0.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        try {
            calibrator.setBiasX(acceleration);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setBiasY(acceleration);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setBiasZ(acceleration);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setBiasCoordinates(0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setBiasCoordinates(acceleration, acceleration, acceleration);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setBias((double[]) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setBias((Matrix) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.calibrate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
    }

    private void assertEstimatedResult(
            final Matrix ma,
            final KnownBiasAndFrameAccelerometerLinearLeastSquaresCalibrator calibrator) {

        assertEquals(ma.getElementAt(0, 0), calibrator.getEstimatedSx(),
                0.0);
        assertEquals(ma.getElementAt(1, 1), calibrator.getEstimatedSy(),
                0.0);
        assertEquals(ma.getElementAt(2, 2), calibrator.getEstimatedSz(),
                0.0);
        assertEquals(ma.getElementAt(0, 1), calibrator.getEstimatedMxy(),
                0.0);
        assertEquals(ma.getElementAt(0, 2), calibrator.getEstimatedMxz(),
                0.0);
        assertEquals(ma.getElementAt(1, 0), calibrator.getEstimatedMyx(),
                0.0);
        assertEquals(ma.getElementAt(1, 2), calibrator.getEstimatedMyz(),
                0.0);
        assertEquals(ma.getElementAt(2, 0), calibrator.getEstimatedMzx(),
                0.0);
        assertEquals(ma.getElementAt(2, 1), calibrator.getEstimatedMzy(),
                0.0);
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
}
