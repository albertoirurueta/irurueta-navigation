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
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import org.junit.Assert;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorTest implements
        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibratorListener {

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

    private static final int LARGE_MEASUREMENT_NUMBER = 100000;

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-5;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;

    private static final int TIMES = 100;

    private int mCalibrateStart;
    private int mCalibrateEnd;

    @Test
    public void testConstructor1() throws WrongSizeException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[3];
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        Assert.assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor2() throws WrongSizeException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(this);

        // check default values
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[3];
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor3() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements);

        // check default values
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[3];
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor4() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                        this);

        // check default values
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[3];
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor5() throws WrongSizeException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        true);

        // check default values
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[3];
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor6() throws WrongSizeException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        true, this);

        // check default values
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[3];
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor7() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                        true);

        // check default values
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[3];
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor8() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                        true, this);

        // check default values
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[3];
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor9() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(biasX,
                        biasY, biasZ);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor10() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(biasX,
                        biasY, biasZ, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor11() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                        biasX, biasY, biasZ);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor12() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                        biasX, biasY, biasZ, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor13() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(biasX,
                        biasY, biasZ, true);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor14() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(biasX,
                        biasY, biasZ, true, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor15() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                        biasX, biasY, biasZ, true);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor16() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                        biasX, biasY, biasZ, true, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor17() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(baX,
                        baY, baZ);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor18() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(baX,
                        baY, baZ, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor19() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                        baX, baY, baZ);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor20() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                        baX, baY, baZ, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor21() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        baX, baY, baZ, true);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor22() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        baX, baY, baZ, true, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor23() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                        baX, baY, baZ, true);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor24() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                        baX, baY, baZ, true, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor25() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        biasX, biasY, biasZ, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor26() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                        biasX, biasY, biasZ, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor27() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(measurements,
                        biasX, biasY, biasZ, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor28() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        true, biasX, biasY, biasZ,
                        initialSx, initialSy, initialSz);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor29() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        true, biasX, biasY, biasZ,
                        initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor30() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, true, biasX, biasY, biasZ,
                        initialSx, initialSy, initialSz);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor31() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, true, biasX, biasY, biasZ,
                        initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor32() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        baX, baY, baZ, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor33() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        baX, baY, baZ, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor34() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, baX, baY, baZ, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor35() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, baX, baY, baZ, initialSx, initialSy, initialSz,
                        this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor36() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        baX, baY, baZ, true,
                        initialSx, initialSy, initialSz);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor37() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, baX, baY, baZ, true,
                        initialSx, initialSy, initialSz);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor38() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, baX, baY, baZ, true,
                        initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor39() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        biasX, biasY, biasZ, initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor40() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, biasX, biasY, biasZ,
                        initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor41() throws WrongSizeException {

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        biasX, biasY, biasZ, true,
                        initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor42() throws WrongSizeException {

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        biasX, biasY, biasZ, true,
                        initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor43() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, biasX, biasY, biasZ, true,
                        initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor44() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, biasX, biasY, biasZ, true,
                        initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor45() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        baX, baY, baZ, initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor46() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        baX, baY, baZ, initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor47() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, baX, baY, baZ, initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor48() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, baX, baY, baZ, initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor49() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        baX, baY, baZ, true, initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor50() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        baX, baY, baZ, true,
                        initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor51() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, baX, baY, baZ, true,
                        initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor52() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        final Acceleration baX = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baY = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baZ = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, baX, baY, baZ, true,
                        initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor53() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final double[] bias = new double[] { biasX, biasY, biasZ };

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        bias);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor54() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final double[] bias = new double[] { biasX, biasY, biasZ };

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        bias, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor55() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final double[] bias = new double[] { biasX, biasY, biasZ };

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, bias);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor56() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final double[] bias = new double[] { biasX, biasY, biasZ };

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, bias, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor57() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final double[] bias = new double[] { biasX, biasY, biasZ };

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        bias, true);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new double[1], true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor58() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final double[] bias = new double[] { biasX, biasY, biasZ };

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        bias, true, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new double[1], true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor59() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final double[] bias = new double[] { biasX, biasY, biasZ };

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, bias, true);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new double[1], true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor60() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final double[] bias = new double[] { biasX, biasY, biasZ };

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, bias, true, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new double[1], true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor61() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(ba);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor62() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(ba,
                        this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new Matrix(1, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new Matrix(3, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor63() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, ba);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor64() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, ba, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor65() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(ba,
                        true);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new Matrix(1, 1), true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new Matrix(3, 3), true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor66() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(ba,
                        true, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new Matrix(1, 1), true, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new Matrix(3, 3), true, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor67() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, ba, true);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(1, 1), true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(3, 3), true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor68() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, ba, true, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(1, 1), true,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(3, 3), true,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor69() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        ba, ma);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new Matrix(1, 1), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new Matrix(3, 3), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    ba, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    ba, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor70() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        ba, ma, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new Matrix(1, 1), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new Matrix(3, 3), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    ba, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    ba, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor71() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, ba, ma);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(1, 1), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(3, 3), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, ba, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, ba, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor72() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, ba, ma, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameAccelerometerNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(1, 1), ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(3, 3), ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, ba, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, ba, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor73() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        ba, true, ma);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new Matrix(1, 1), true, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new Matrix(3, 3), true, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    ba, true, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    ba, true, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor74() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        ba, true, ma, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new Matrix(1, 1), true, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    new Matrix(3, 3), true, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    ba, true, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    ba, true, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor75() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, ba, true, ma);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(1, 1), true,
                    ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(3, 3), true,
                    ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, ba, true,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, ba, true,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor76() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, ba, true, ma, this);

        // check default values
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = new double[] { biasX, biasY, biasZ };
        assertArrayEquals(calibrator.getBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = Matrix.newFromArray(bias1);
        assertEquals(calibrator.getBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix ma1 = new Matrix(3, 3);
        ma1.setSubmatrix(0, 0,
                2, 2,
                new double[]{ initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(calibrator.getInitialMa(), ma1);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
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
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(1, 1), true,
                    ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(3, 3), true,
                    ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, ba, true,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                    measurements, ba, true,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testGetSetBiasX() throws LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);

        calibrator.setBiasX(biasX);

        // check
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
    }

    @Test
    public void testGetSetBiasY() throws LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double biasY = ba.getElementAtIndex(1);

        calibrator.setBiasY(biasY);

        // check
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
    }

    @Test
    public void testGetSetBiasZ() throws LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double biasZ = ba.getElementAtIndex(2);

        calibrator.setBiasZ(biasZ);

        // check
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
    }

    @Test
    public void testGetSetBiasXAsAcceleration() throws LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        final Acceleration biasX1 = calibrator.getBiasXAsAcceleration();
        assertEquals(biasX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(biasX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // set new value
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final Acceleration biasX2 = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setBiasX(biasX2);

        // check
        final Acceleration biasX3 = calibrator.getBiasXAsAcceleration();
        final Acceleration biasX4 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(biasX4);

        assertEquals(biasX2, biasX3);
        assertEquals(biasX2, biasX4);
    }

    @Test
    public void testGetSetBiasYAsAcceleration() throws LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        final Acceleration biasY1 = calibrator.getBiasYAsAcceleration();
        assertEquals(biasY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(biasY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // set new value
        final Matrix ba = generateBa();
        final double biasY = ba.getElementAtIndex(1);
        final Acceleration biasY2 = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setBiasY(biasY2);

        // check
        final Acceleration biasY3 = calibrator.getBiasYAsAcceleration();
        final Acceleration biasY4 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(biasY4);

        assertEquals(biasY2, biasY3);
        assertEquals(biasY2, biasY4);
    }

    @Test
    public void testGetSetBiasZAsAcceleration() throws LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        final Acceleration biasZ1 = calibrator.getBiasZAsAcceleration();
        assertEquals(biasZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(biasZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // set new value
        final Matrix ba = generateBa();
        final double biasZ = ba.getElementAtIndex(2);
        final Acceleration biasZ2 = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setBiasZ(biasZ2);

        // check
        final Acceleration biasZ3 = calibrator.getBiasZAsAcceleration();
        final Acceleration biasZ4 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(biasZ4);

        assertEquals(biasZ2, biasZ3);
        assertEquals(biasZ2, biasZ4);
    }

    @Test
    public void testSetBias1() throws LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);

        // set new values
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        calibrator.setBiasCoordinates(biasX, biasY, biasZ);

        // check
        assertEquals(biasX, calibrator.getBiasX(), 0.0);
        assertEquals(biasY, calibrator.getBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getBiasZ(), 0.0);
    }

    @Test
    public void testSetBias2() throws LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);

        // set new values
        final Matrix ba = generateBa();
        final Acceleration biasX = new Acceleration(ba.getElementAtIndex(0),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration biasY = new Acceleration(ba.getElementAtIndex(1),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration biasZ = new Acceleration(ba.getElementAtIndex(2),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setBiasCoordinates(biasX, biasY, biasZ);

        // check
        assertEquals(biasX, calibrator.getBiasXAsAcceleration());
        assertEquals(biasY, calibrator.getBiasYAsAcceleration());
        assertEquals(biasZ, calibrator.getBiasZAsAcceleration());
    }

    @Test
    public void testGetSetInitialSx() throws WrongSizeException, LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);

        calibrator.setInitialSx(initialSx);

        // check
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
    }

    @Test
    public void testGetSetInitialSy() throws WrongSizeException, LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double initialSy = ma.getElementAt(1, 1);

        calibrator.setInitialSy(initialSy);

        // check
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
    }

    @Test
    public void testGetSetInitialSz() throws WrongSizeException, LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double initialSz = ma.getElementAt(2, 2);

        calibrator.setInitialSz(initialSz);

        // check
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
    }

    @Test
    public void testGetSetInitialMxy() throws WrongSizeException, LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double initialMxy = ma.getElementAt(0, 1);

        calibrator.setInitialMxy(initialMxy);

        // check
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
    }

    @Test
    public void testGetSetInitialMxz() throws WrongSizeException, LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double initialMxz = ma.getElementAt(0, 2);

        calibrator.setInitialMxz(initialMxz);

        // check
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
    }

    @Test
    public void testGetSetInitialMyx() throws WrongSizeException, LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double initialMyx = ma.getElementAt(1, 0);

        calibrator.setInitialMyx(initialMyx);

        // check
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
    }

    @Test
    public void testGetSetInitialMyz() throws WrongSizeException, LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double initialMyz = ma.getElementAt(1, 2);

        calibrator.setInitialMyz(initialMyz);

        // check
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
    }

    @Test
    public void testGetSetInitialMzx() throws WrongSizeException, LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double initialMzx = ma.getElementAt(2, 0);

        calibrator.setInitialMzx(initialMzx);

        // check
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
    }

    @Test
    public void testGetSetInitialMzy() throws WrongSizeException, LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double initialMzy = ma.getElementAt(2, 1);

        calibrator.setInitialMzy(initialMzy);

        // check
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
    }

    @Test
    public void testSetInitialScalingFactors() throws WrongSizeException,
            LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);

        // set new values
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);

        calibrator.setInitialScalingFactors(initialSx, initialSy, initialSz);

        // check
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
    }

    @Test
    public void testSetInitialScalingFactorsAndCrossCouplingErrors()
            throws WrongSizeException, LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        // set new values
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);
        final double initialSy = ma.getElementAt(1, 1);
        final double initialSz = ma.getElementAt(2, 2);
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);

        // check
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
    }

    @Test
    public void testGetSetBiasAsArray() throws LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

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
    public void testGetSetBiasAsMatrix() throws LockedException, WrongSizeException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

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
    public void testGetSetInitialMa() throws WrongSizeException, LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        final Matrix ma1 = calibrator.getInitialMa();
        final Matrix ma2 = new Matrix(BodyKinematics.COMPONENTS,
                BodyKinematics.COMPONENTS);
        calibrator.getInitialMa(ma2);

        assertEquals(ma1, new Matrix(BodyKinematics.COMPONENTS,
                BodyKinematics.COMPONENTS));
        assertEquals(ma1, ma2);

        // set new value
        final Matrix ma3 = generateMaGeneral();
        calibrator.setInitialMa(ma3);

        // check
        final Matrix ma4 = calibrator.getInitialMa();
        final Matrix ma5 = new Matrix(BodyKinematics.COMPONENTS,
                BodyKinematics.COMPONENTS);
        calibrator.getInitialMa(ma5);

        assertEquals(ma3, ma4);
        assertEquals(ma3, ma5);

        // Force IllegalArgumentException
        try {
            calibrator.getInitialMa(new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.getInitialMa(new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setInitialMa(new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setInitialMa(new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMeasurements() throws LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getMeasurements());

        // set new value
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        calibrator.setMeasurements(measurements);

        // check
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    public void testIsSetCommonAxisUsed() throws LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testIsReady() throws LockedException {
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertFalse(calibrator.isReady());
        assertNull(calibrator.getMeasurements());

        // set enough measurements
        calibrator.setMeasurements(Arrays.asList(
                new StandardDeviationFrameBodyKinematics(),
                new StandardDeviationFrameBodyKinematics(),
                new StandardDeviationFrameBodyKinematics()));

        // check
        assertTrue(calibrator.isReady());
        assertNotNull(calibrator.getMeasurements());
    }

    @Test
    public void testCalibrateMultipleOrientationsForGeneralCaseWithMinimumMeasuresAndNoNoise()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, CalibrationException {
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

        final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
        final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
        final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

        final List<StandardDeviationFrameBodyKinematics> measurements = new ArrayList<>();
        for (int i = 0; i < KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

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

            final StandardDeviationFrameBodyKinematics measurement =
                    new StandardDeviationFrameBodyKinematics(measuredKinematics,
                            ecefFrame, ecefFrame, TIME_INTERVAL_SECONDS,
                            specificForceStandardDeviation,
                            angularRateStandardDeviation);
            measurements.add(measurement);
        }

        // When we have the minimum number of measurements, we need to provide
        // an initial solution close to the true solution
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, ba, false, ma, this);

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

        assertNotNull(calibrator.getEstimatedCovariance());
        assertTrue(calibrator.getEstimatedChiSq() < 0.0);
    }

    @Test
    public void testCalibrateMultipleOrientationsForGeneralCaseWithNoiseLargeNumberOfMeasurements()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, CalibrationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMaGeneral();
            final Matrix mg = generateMg();
            final Matrix gg = generateGg();
            // when using minimum number of measurements we must not add any noise so that
            // a solution is found. When adding more measurements, certain noise can be added
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

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final List<StandardDeviationFrameBodyKinematics> measurements = new ArrayList<>();
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

                final StandardDeviationFrameBodyKinematics measurement =
                        new StandardDeviationFrameBodyKinematics(measuredKinematics,
                                ecefFrame, ecefFrame, TIME_INTERVAL_SECONDS,
                                specificForceStandardDeviation,
                                angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have a large number of measurements, there is no need to provide
            // an initial value because the default initial value (all zeros) will
            // typically converge close to the true solution
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                    new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                            measurements, ba, false, this);

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

            assertNotNull(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedChiSq() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultiplePositionsForGeneralCaseWithMinimumMeasuresAndNoNoise()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, CalibrationException {
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

        final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
        final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
        final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

        final List<StandardDeviationFrameBodyKinematics> measurements = new ArrayList<>();
        for (int i = 0; i < KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

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

            final StandardDeviationFrameBodyKinematics measurement =
                    new StandardDeviationFrameBodyKinematics(measuredKinematics,
                            ecefFrame, ecefFrame, TIME_INTERVAL_SECONDS,
                            specificForceStandardDeviation,
                            angularRateStandardDeviation);
            measurements.add(measurement);
        }

        // When we have the minimum number of measurements, we need to provide
        // an initial solution close to the true solution
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, ba, false, ma, this);

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

        assertNotNull(calibrator.getEstimatedCovariance());
    }

    @Test
    public void testCalibrateMultipleOrientationsForCommonAxisCaseWithMinimumMeasuresAndNoNoise()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, CalibrationException {
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

        final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
        final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
        final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

        final List<StandardDeviationFrameBodyKinematics> measurements = new ArrayList<>();
        for (int i = 0; i < KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

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

            final StandardDeviationFrameBodyKinematics measurement =
                    new StandardDeviationFrameBodyKinematics(measuredKinematics,
                            ecefFrame, ecefFrame, TIME_INTERVAL_SECONDS,
                            specificForceStandardDeviation,
                            angularRateStandardDeviation);
            measurements.add(measurement);
        }

        // When we have the minimum number of measurements, we need to provide
        // an initial solution close to the true solution
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, ba, true, ma, this);

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

        assertNotNull(calibrator.getEstimatedCovariance());
        assertTrue(calibrator.getEstimatedChiSq() < 0.0);
    }

    @Test
    public void testCalibrateMultipleOrientationsForCommonAxisCaseWithNoiseLargeNumberOfMeasurements()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, CalibrationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMaCommonAxis();
            final Matrix mg = generateMg();
            final Matrix gg = generateGg();
            // when using minimum number of measurements we must not add any noise so that
            // a solution is found. When adding more measurements, certain noise can be added
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

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final List<StandardDeviationFrameBodyKinematics> measurements = new ArrayList<>();
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

                final StandardDeviationFrameBodyKinematics measurement =
                        new StandardDeviationFrameBodyKinematics(measuredKinematics,
                                ecefFrame, ecefFrame, TIME_INTERVAL_SECONDS,
                                specificForceStandardDeviation,
                                angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have a large number of measurements, there is no need to provide
            // an initial value because the default initial value (all zeros) will
            // typically converge close to the true solution
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                    new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                            measurements, true, this);

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

            assertNotNull(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedChiSq() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultiplePositionsForCommonAxisCaseWithMinimumMeasuresAndNoNoise()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, CalibrationException {
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

        final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
        final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
        final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

        final List<StandardDeviationFrameBodyKinematics> measurements = new ArrayList<>();
        for (int i = 0; i < KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

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

            final StandardDeviationFrameBodyKinematics measurement =
                    new StandardDeviationFrameBodyKinematics(measuredKinematics,
                            ecefFrame, ecefFrame, TIME_INTERVAL_SECONDS,
                            specificForceStandardDeviation,
                            angularRateStandardDeviation);
            measurements.add(measurement);
        }

        // When we have the minimum number of measurements, we need to provide
        // an initial solution close to the true solution
        final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator(
                        measurements, ba, true, ma, this);

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

        assertTrue(ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR));

        assertNotNull(calibrator.getEstimatedCovariance());
        assertTrue(calibrator.getEstimatedChiSq() < 0.0);
    }

    @Override
    public void onCalibrateStart(
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        mCalibrateStart++;
    }

    @Override
    public void onCalibrateEnd(
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        mCalibrateEnd++;
    }

    private void reset() {
        mCalibrateStart = 0;
        mCalibrateEnd = 0;
    }

    private void checkLocked(
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
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
        try {
            calibrator.setBiasX(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setBiasY(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setBiasZ(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setBiasCoordinates(0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setBiasCoordinates(null, null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialSx(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialSy(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialSz(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialMxy(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialMxz(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialMyx(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialMyz(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialMzx(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialMzy(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialScalingFactors(0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialCrossCouplingErrors(
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0);
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
            calibrator.setInitialMa(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
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
            calibrator.calibrate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
    }

    private void assertEstimatedResult(
            final Matrix ma,
            final KnownBiasAndFrameAccelerometerNonLinearLeastSquaresCalibrator calibrator) {

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
