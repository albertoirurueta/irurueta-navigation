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
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import org.junit.Assert;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class MSACRobustKnownBiasAndPositionAccelerometerCalibratorTest implements
        RobustKnownBiasAndPositionAccelerometerCalibratorListener {

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

    private static final int MEASUREMENT_NUMBER = 1000;
    private static final int LARGE_MEASUREMENT_NUMBER = 100000;

    private static final int OUTLIER_PERCENTAGE = 10;

    private static final double THRESHOLD = 1e-10;
    private static final double LARGE_THRESHOLD = 1e-2;

    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;

    private static final double OUTLIER_ERROR_FACTOR = 100.0;

    private static final int TIMES = 100;

    private int mCalibrateStart;
    private int mCalibrateEnd;
    private int mCalibrateNextIteration;
    private int mCalibrateProgressChange;

    @Test
    public void testConstructor1() throws WrongSizeException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
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
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getEcefPosition());
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        Assert.assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor2() throws WrongSizeException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
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
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getEcefPosition());
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor3() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(measurements);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
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
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getEcefPosition());
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor4() throws WrongSizeException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(true);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
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
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getEcefPosition());
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor5() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(bias);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getEcefPosition());
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor6() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(ba);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getEcefPosition());
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor7() throws WrongSizeException {
        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        final Matrix ma = generateMaGeneral();
        final double sx = ma.getElementAt(0, 0);
        final double sy = ma.getElementAt(1, 1);
        final double sz = ma.getElementAt(2, 2);
        final double mxy = ma.getElementAt(0, 1);
        final double mxz = ma.getElementAt(0, 2);
        final double myx = ma.getElementAt(1, 0);
        final double myz = ma.getElementAt(1, 2);
        final double mzx = ma.getElementAt(2, 0);
        final double mzy = ma.getElementAt(2, 1);

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(ba, ma);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(calibrator.getInitialMa(), ma);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertNull(calibrator.getEcefPosition());
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    new Matrix(1, 1), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    new Matrix(3, 3), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ba, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ba, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor8() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(ecefPosition);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
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
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor9() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(ecefPosition,
                        measurements);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
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
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor10() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(ecefPosition,
                        measurements, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
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
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor11() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(ecefPosition,
                        measurements, true);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
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
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor12() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(ecefPosition,
                        measurements, true, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
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
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor13() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(ecefPosition,
                        measurements, bias);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor14() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(ecefPosition,
                        measurements, bias, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor15() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(ecefPosition,
                        measurements, true, bias);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, true, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor16() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(ecefPosition,
                        measurements, true, bias, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, true, new double[1],
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor17() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(ecefPosition,
                        measurements, ba);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor18() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(ecefPosition,
                        measurements, ba, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, new Matrix(1, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, new Matrix(3, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor19() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(ecefPosition,
                        measurements, true, ba);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, true,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, true,
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor20() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(ecefPosition,
                        measurements, true, ba, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, true,
                    new Matrix(1, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, true,
                    new Matrix(3, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor21() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        final Matrix ma = generateMaGeneral();
        final double sx = ma.getElementAt(0, 0);
        final double sy = ma.getElementAt(1, 1);
        final double sz = ma.getElementAt(2, 2);
        final double mxy = ma.getElementAt(0, 1);
        final double mxz = ma.getElementAt(0, 2);
        final double myx = ma.getElementAt(1, 0);
        final double myz = ma.getElementAt(1, 2);
        final double mzx = ma.getElementAt(2, 0);
        final double mzy = ma.getElementAt(2, 1);

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(ecefPosition,
                        measurements, ba, ma);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(calibrator.getInitialMa(), ma);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, new Matrix(1, 1), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, new Matrix(3, 3), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, ba, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, ba, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor22() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        final Matrix ma = generateMaGeneral();
        final double sx = ma.getElementAt(0, 0);
        final double sy = ma.getElementAt(1, 1);
        final double sz = ma.getElementAt(2, 2);
        final double mxy = ma.getElementAt(0, 1);
        final double mxz = ma.getElementAt(0, 2);
        final double myx = ma.getElementAt(1, 0);
        final double myz = ma.getElementAt(1, 2);
        final double mzx = ma.getElementAt(2, 0);
        final double mzy = ma.getElementAt(2, 1);

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(ecefPosition,
                        measurements, ba, ma, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(calibrator.getInitialMa(), ma);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, new Matrix(1, 1), ma,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, new Matrix(3, 3), ma,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, ba, new Matrix(1, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, ba, new Matrix(3, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor23() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        final Matrix ma = generateMaGeneral();
        final double sx = ma.getElementAt(0, 0);
        final double sy = ma.getElementAt(1, 1);
        final double sz = ma.getElementAt(2, 2);
        final double mxy = ma.getElementAt(0, 1);
        final double mxz = ma.getElementAt(0, 2);
        final double myx = ma.getElementAt(1, 0);
        final double myz = ma.getElementAt(1, 2);
        final double mzx = ma.getElementAt(2, 0);
        final double mzy = ma.getElementAt(2, 1);

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(ecefPosition,
                        measurements, true, ba, ma);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(calibrator.getInitialMa(), ma);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, true, new Matrix(1, 1),
                    ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, true, new Matrix(
                    3, 3), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, true, ba,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, true, ba,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor24() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        final Matrix ma = generateMaGeneral();
        final double sx = ma.getElementAt(0, 0);
        final double sy = ma.getElementAt(1, 1);
        final double sz = ma.getElementAt(2, 2);
        final double mxy = ma.getElementAt(0, 1);
        final double mxz = ma.getElementAt(0, 2);
        final double myx = ma.getElementAt(1, 0);
        final double myz = ma.getElementAt(1, 2);
        final double mzx = ma.getElementAt(2, 0);
        final double mzy = ma.getElementAt(2, 1);

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(ecefPosition,
                        measurements, true, ba, ma, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(calibrator.getInitialMa(), ma);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, true,
                    new Matrix(1, 1), ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, true,
                    new Matrix(3, 3), ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, true, ba,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    ecefPosition, measurements, true, ba,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor25() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(nedPosition);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
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
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertNull(calibrator.getMeasurements());
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor26() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(nedPosition,
                        measurements);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
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
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor27() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(nedPosition,
                        measurements, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
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
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor28() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(nedPosition,
                        measurements, true);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
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
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor29() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(nedPosition,
                        measurements, true, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
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
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor30() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(nedPosition,
                        measurements, bias);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor31() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(nedPosition,
                        measurements, bias, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor32() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(nedPosition,
                        measurements, true, bias);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, true, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor33() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(nedPosition,
                        measurements, true, bias, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, true, new double[1],
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor34() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(nedPosition,
                        measurements, ba);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor35() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(nedPosition,
                        measurements, ba, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, new Matrix(1, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, new Matrix(3, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor36() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(nedPosition,
                        measurements, true, ba);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, true,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, true,
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor37() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(nedPosition,
                        measurements, true, ba, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, true,
                    new Matrix(1, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, true,
                    new Matrix(3, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor38() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        final Matrix ma = generateMaGeneral();
        final double sx = ma.getElementAt(0, 0);
        final double sy = ma.getElementAt(1, 1);
        final double sz = ma.getElementAt(2, 2);
        final double mxy = ma.getElementAt(0, 1);
        final double mxz = ma.getElementAt(0, 2);
        final double myx = ma.getElementAt(1, 0);
        final double myz = ma.getElementAt(1, 2);
        final double mzx = ma.getElementAt(2, 0);
        final double mzy = ma.getElementAt(2, 1);

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(nedPosition,
                        measurements, ba, ma);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(calibrator.getInitialMa(), ma);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, new Matrix(1, 1), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, new Matrix(3, 3), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, ba, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, ba, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor39() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        final Matrix ma = generateMaGeneral();
        final double sx = ma.getElementAt(0, 0);
        final double sy = ma.getElementAt(1, 1);
        final double sz = ma.getElementAt(2, 2);
        final double mxy = ma.getElementAt(0, 1);
        final double mxz = ma.getElementAt(0, 2);
        final double myx = ma.getElementAt(1, 0);
        final double myz = ma.getElementAt(1, 2);
        final double mzx = ma.getElementAt(2, 0);
        final double mzy = ma.getElementAt(2, 1);

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(nedPosition,
                        measurements, ba, ma, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(calibrator.getInitialMa(), ma);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, new Matrix(1, 1), ma,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, new Matrix(3, 3), ma,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, ba, new Matrix(1, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, ba, new Matrix(3, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor40() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        final Matrix ma = generateMaGeneral();
        final double sx = ma.getElementAt(0, 0);
        final double sy = ma.getElementAt(1, 1);
        final double sz = ma.getElementAt(2, 2);
        final double mxy = ma.getElementAt(0, 1);
        final double mxz = ma.getElementAt(0, 2);
        final double myx = ma.getElementAt(1, 0);
        final double myz = ma.getElementAt(1, 2);
        final double mzx = ma.getElementAt(2, 0);
        final double mzy = ma.getElementAt(2, 1);

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(nedPosition,
                        measurements, true, ba, ma);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(calibrator.getInitialMa(), ma);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, true,
                    new Matrix(1, 1), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, true,
                    new Matrix(3, 3), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, true, ba,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, true, ba,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor41() throws WrongSizeException {
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final double[] bias = ba.getBuffer();

        final Matrix ma = generateMaGeneral();
        final double sx = ma.getElementAt(0, 0);
        final double sy = ma.getElementAt(1, 1);
        final double sz = ma.getElementAt(2, 2);
        final double mxy = ma.getElementAt(0, 1);
        final double mxz = ma.getElementAt(0, 2);
        final double myx = ma.getElementAt(1, 0);
        final double myz = ma.getElementAt(1, 2);
        final double mzx = ma.getElementAt(2, 0);
        final double mzy = ma.getElementAt(2, 1);

        MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(nedPosition,
                        measurements, true, ba, ma, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.MSAC);
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
        Acceleration acceleration1 = calibrator.getBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bx, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), by, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), bz, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        final AccelerationTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bx, 0.0);
        assertEquals(biasTriad1.getValueY(), by, 0.0);
        assertEquals(biasTriad1.getValueZ(), bz, 0.0);
        assertEquals(biasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad biasTriad2 = new AccelerationTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getBias(), bias, 0.0);
        double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias, bias2, 0.0);
        assertEquals(calibrator.getBiasAsMatrix(), ba);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(biasMatrix2);
        assertEquals(ba, biasMatrix2);
        assertEquals(calibrator.getInitialMa(), ma);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma, ma2);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, ABSOLUTE_ERROR));
        final NEDPosition nedPosition1 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition1));
        assertTrue(nedPosition1.equals(nedPosition, ABSOLUTE_ERROR));
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS,
                calibrator.getMeasurementType());
        assertTrue(calibrator.isOrderedMeasurementsRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, true,
                    new Matrix(1, 1), ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, true,
                    new Matrix(3, 3), ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, true, ba,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                    nedPosition, measurements, true, ba,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertEquals(calibrator.getThreshold(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.DEFAULT_THRESHOLD, 0.0);

        // set new value
        calibrator.setThreshold(0.1);

        // check
        assertEquals(calibrator.getThreshold(), 0.1, 0.0);

        // Force IllegalArgumentException
        try {
            calibrator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetInitialBiasX() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

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
    public void testGetSetInitialBiasY() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

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
    public void testGetSetInitialBiasZ() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

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
    public void testGetSetInitialBiasXAsAcceleration() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

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
    public void testGetSetInitialBiasYAsAcceleration() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

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
    public void testGetSetInitialBiasZAsAcceleration() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

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
    public void testSetBiasCoordinates1() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default values
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);

        // set new value
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
    public void testSetBiasCoordinates2() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default values
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final Acceleration bax = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration bay = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baz = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setBiasCoordinates(bax, bay, baz);

        // check
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
    }

    @Test
    public void testGetSetBiasAsTriad() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default values
        final AccelerationTriad triad1 = calibrator.getBiasAsTriad();
        assertEquals(triad1.getValueX(), 0.0, 0.0);
        assertEquals(triad1.getValueY(), 0.0, 0.0);
        assertEquals(triad1.getValueZ(), 0.0, 0.0);
        assertEquals(triad1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // set new values
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        final AccelerationTriad triad2 = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND,
                biasX, biasY, biasZ);
        calibrator.setBias(triad2);

        // check
        final AccelerationTriad triad3 = calibrator.getBiasAsTriad();
        final AccelerationTriad triad4 = new AccelerationTriad();
        calibrator.getBiasAsTriad(triad4);

        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    public void testGetSetInitialSx() throws WrongSizeException, LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double initialSx = ma.getElementAt(0, 0);

        calibrator.setInitialSx(initialSx);

        // check
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
    }

    @Test
    public void testGetSetInitialSy() throws WrongSizeException, LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double initialSy = ma.getElementAt(1, 1);

        calibrator.setInitialSy(initialSy);

        // check
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
    }

    @Test
    public void testGetSetInitialSz() throws WrongSizeException, LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double initialSz = ma.getElementAt(2, 2);

        calibrator.setInitialSz(initialSz);

        // check
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
    }

    @Test
    public void testGetSetInitialMxy() throws WrongSizeException, LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double initialMxy = ma.getElementAt(0, 1);

        calibrator.setInitialMxy(initialMxy);

        // check
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
    }

    @Test
    public void testGetSetInitialMxz() throws WrongSizeException, LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double initialMxz = ma.getElementAt(0, 2);

        calibrator.setInitialMxz(initialMxz);

        // check
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
    }

    @Test
    public void testGetSetInitialMyx() throws WrongSizeException, LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double initialMyx = ma.getElementAt(1, 0);

        calibrator.setInitialMyx(initialMyx);

        // check
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
    }

    @Test
    public void testGetSetInitialMyz() throws WrongSizeException, LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double initialMyz = ma.getElementAt(1, 2);

        calibrator.setInitialMyz(initialMyz);

        // check
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
    }

    @Test
    public void testGetSetInitialMzx() throws WrongSizeException, LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double initialMzx = ma.getElementAt(2, 0);

        calibrator.setInitialMzx(initialMzx);

        // check
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
    }

    @Test
    public void testGetSetInitialMzy() throws WrongSizeException, LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double initialMzy = ma.getElementAt(2, 1);

        calibrator.setInitialMzy(initialMzy);

        // check
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
    }

    @Test
    public void testGetSetInitialScalingFactors() throws WrongSizeException, LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

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
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
    }

    @Test
    public void testGetSetInitialCrossCouplingErrors() throws WrongSizeException, LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default values
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        // set new values
        final Matrix ma = generateMaGeneral();
        final double initialMxy = ma.getElementAt(0, 1);
        final double initialMxz = ma.getElementAt(0, 2);
        final double initialMyx = ma.getElementAt(1, 0);
        final double initialMyz = ma.getElementAt(1, 2);
        final double initialMzx = ma.getElementAt(2, 0);
        final double initialMzy = ma.getElementAt(2, 1);

        calibrator.setInitialCrossCouplingErrors(initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);

        // check
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
    }

    @Test
    public void testSetInitialScalingFactorsAndCrossCouplingErrors()
            throws WrongSizeException, LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

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
    public void testGetSetInitialBias() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, new double[3], 0.0);

        // set new value
        final double[] bias2 = generateBa().getBuffer();
        calibrator.setBias(bias2);

        // check
        final double[] bias3 = calibrator.getBias();
        final double[] bias4 = new double[3];
        calibrator.getBias(bias4);

        assertArrayEquals(bias2, bias3, 0.0);
        assertArrayEquals(bias2, bias4, 0.0);

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
    public void testGetSetInitialBiasAsMatrix() throws LockedException, WrongSizeException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        final Matrix bias1 = calibrator.getBiasAsMatrix();
        assertArrayEquals(bias1.getBuffer(), new double[3], 0.0);

        // set new values
        final Matrix bias2 = generateBa();
        calibrator.setBias(bias2);

        // check
        final Matrix bias3 = calibrator.getBiasAsMatrix();
        final Matrix bias4 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bias4);

        assertEquals(bias2, bias3);
        assertEquals(bias2, bias4);

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
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        final Matrix ma1 = calibrator.getInitialMa();
        assertEquals(ma1, new Matrix(3, 3));

        // set new value
        final Matrix ma2 = generateMaGeneral();
        calibrator.setInitialMa(ma2);

        final double initialSx = ma2.getElementAt(0, 0);
        final double initialSy = ma2.getElementAt(1, 1);
        final double initialSz = ma2.getElementAt(2, 2);
        final double initialMxy = ma2.getElementAt(0, 1);
        final double initialMxz = ma2.getElementAt(0, 2);
        final double initialMyx = ma2.getElementAt(1, 0);
        final double initialMyz = ma2.getElementAt(1, 2);
        final double initialMzx = ma2.getElementAt(2, 0);
        final double initialMzy = ma2.getElementAt(2, 1);


        // check
        final Matrix ma3 = calibrator.getInitialMa();
        final Matrix ma4 = new Matrix(3, 3);
        calibrator.getInitialMa(ma4);

        assertEquals(ma2, ma3);
        assertEquals(ma2, ma3);

        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);

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
    public void testGetSetEcefPosition() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertNull(calibrator.getEcefPosition());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        calibrator.setPosition(ecefPosition);

        // check
        assertSame(calibrator.getEcefPosition(), ecefPosition);
    }

    @Test
    public void testGetSetNedPosition() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition1 = new NEDPosition(latitude, longitude, height);

        calibrator.setPosition(nedPosition1);

        // check
        final NEDPosition nedPosition2 = calibrator.getNedPosition();
        final NEDPosition nedPosition3 = new NEDPosition();
        calibrator.getNedPosition(nedPosition3);

        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(nedPosition1.equals(nedPosition3, ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetMeasurements() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertNull(calibrator.getMeasurements());

        // set new value
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        calibrator.setMeasurements(measurements);

        // check
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testIsSetCommonAxisUsed() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testIsReady() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check
        assertFalse(calibrator.isReady());


        // set empty measurements
        final List<StandardDeviationBodyKinematics> measurements1 =
                Collections.emptyList();
        calibrator.setMeasurements(measurements1);

        // check
        assertFalse(calibrator.isReady());


        // set enough measurements for general case
        calibrator.setCommonAxisUsed(false);

        final List<StandardDeviationBodyKinematics> measurements2 = new ArrayList<>();
        for (int i = 0; i < KnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL; i++) {
            measurements2.add(new StandardDeviationBodyKinematics());
        }
        calibrator.setMeasurements(measurements2);

        // check
        assertFalse(calibrator.isReady());


        // set position
        final ECEFPosition position = new ECEFPosition();
        calibrator.setPosition(position);

        assertTrue(calibrator.isReady());


        // set enough measurements for common axis case
        measurements2.clear();
        for (int i = 0; i < KnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_COMON_Z_AXIS; i++) {
            measurements2.add(new StandardDeviationBodyKinematics());
        }
        calibrator.setMeasurements(measurements2);

        // check
        assertFalse(calibrator.isReady());

        calibrator.setCommonAxisUsed(true);

        assertTrue(calibrator.isReady());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertEquals(calibrator.getProgressDelta(), 0.05f, 0.0);

        // set new value
        calibrator.setProgressDelta(0.01f);

        // check
        assertEquals(calibrator.getProgressDelta(), 0.01f, 0.0);

        // force IllegalArgumentException
        try {
            calibrator.setProgressDelta(-1.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setProgressDelta(2.0f);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetConfidence() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertEquals(calibrator.getConfidence(), 0.99, 0.0);

        // set new value
        calibrator.setConfidence(0.5);

        // check
        assertEquals(calibrator.getConfidence(), 0.5, 0.0);

        // Force IllegalArgumentException
        try {
            calibrator.setConfidence(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setConfidence(2.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMaxIterations() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertEquals(calibrator.getMaxIterations(), 5000);

        // set new value
        calibrator.setMaxIterations(100);

        assertEquals(calibrator.getMaxIterations(), 100);

        // Force IllegalArgumentException
        try {
            calibrator.setMaxIterations(0);
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertTrue(calibrator.isResultRefined());

        // set new value
        calibrator.setResultRefined(false);

        // check
        assertFalse(calibrator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertTrue(calibrator.isCovarianceKept());

        // set new value
        calibrator.setCovarianceKept(false);

        // check
        assertFalse(calibrator.isCovarianceKept());
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertNull(calibrator.getQualityScores());

        // set new value
        calibrator.setQualityScores(new double[3]);

        // check
        assertNull(calibrator.getQualityScores());
    }

    @Test
    public void testGetSetPreliminarySubsetSize() throws LockedException {
        final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                new MSACRobustKnownBiasAndPositionAccelerometerCalibrator();

        // check default value
        assertEquals(calibrator.getPreliminarySubsetSize(),
                MSACRobustKnownBiasAndPositionAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL);

        // set new value
        calibrator.setPreliminarySubsetSize(14);

        // check
        assertEquals(calibrator.getPreliminarySubsetSize(), 14);

        // Force IllegalArgumentException
        try {
            calibrator.setPreliminarySubsetSize(9);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testCalibrateGeneralNoNoiseInlier() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            CalibrationException, NotReadyException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMaGeneral();
            final Matrix mg = generateMg();
            final Matrix gg = generateGg();
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg,
                    OUTLIER_ERROR_FACTOR * accelNoiseRootPSD,
                    OUTLIER_ERROR_FACTOR * gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);
            final IMUErrors errorsInlier = new IMUErrors(ba, bg, ma, mg, gg,
                    0.0, 0.0, accelQuantLevel,
                    gyroQuantLevel);


            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD()
                    / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD()
                    / sqrtTimeInterval;

            final List<StandardDeviationBodyKinematics> measurements =
                    new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
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
                final BodyKinematics measuredKinematics;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                    errorsOutlier, random);

                } else {
                    // inlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                    errorsInlier, random);
                }

                final StandardDeviationBodyKinematics measurement =
                        new StandardDeviationBodyKinematics(measuredKinematics,
                                specificForceStandardDeviation,
                                angularRateStandardDeviation);
                measurements.add(measurement);
            }

            final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                    new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(nedPosition, measurements,
                            false, ba, ma, this);
            calibrator.setThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);
            assertEquals(mCalibrateNextIteration, 0);
            assertEquals(mCalibrateProgressChange, 0);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedMa = calibrator.getEstimatedMa();

            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMa, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(calibrator.getEstimatedChiSq(), 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateCommonAxisNoNoiseInlier() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            CalibrationException, NotReadyException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMaCommonAxis();
            final Matrix mg = generateMg();
            final Matrix gg = generateGg();
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg,
                    OUTLIER_ERROR_FACTOR * accelNoiseRootPSD,
                    OUTLIER_ERROR_FACTOR * gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);
            final IMUErrors errorsInlier = new IMUErrors(ba, bg, ma, mg, gg,
                    0.0, 0.0, accelQuantLevel,
                    gyroQuantLevel);


            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD()
                    / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD()
                    / sqrtTimeInterval;

            final List<StandardDeviationBodyKinematics> measurements =
                    new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
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
                final BodyKinematics measuredKinematics;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                    errorsOutlier, random);

                } else {
                    // inlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                    errorsInlier, random);
                }

                final StandardDeviationBodyKinematics measurement =
                        new StandardDeviationBodyKinematics(measuredKinematics,
                                specificForceStandardDeviation,
                                angularRateStandardDeviation);
                measurements.add(measurement);
            }

            final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                    new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(nedPosition, measurements,
                            true, ba, ma, this);
            calibrator.setThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);
            assertEquals(mCalibrateNextIteration, 0);
            assertEquals(mCalibrateProgressChange, 0);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedMa = calibrator.getEstimatedMa();

            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMa, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(calibrator.getEstimatedChiSq(), 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralWithInlierNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            CalibrationException, NotReadyException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMaGeneral();
            final Matrix mg = generateMg();
            final Matrix gg = generateGg();
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg,
                    OUTLIER_ERROR_FACTOR * accelNoiseRootPSD,
                    OUTLIER_ERROR_FACTOR * gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);
            final IMUErrors errorsInlier = new IMUErrors(ba, bg, ma, mg, gg,
                    accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                    gyroQuantLevel);


            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD()
                    / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD()
                    / sqrtTimeInterval;

            final List<StandardDeviationBodyKinematics> measurements =
                    new ArrayList<>();
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
                final BodyKinematics measuredKinematics;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                    errorsOutlier, random);

                } else {
                    // inlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                    errorsInlier, random);
                }

                final StandardDeviationBodyKinematics measurement =
                        new StandardDeviationBodyKinematics(measuredKinematics,
                                specificForceStandardDeviation,
                                angularRateStandardDeviation);
                measurements.add(measurement);
            }

            final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                    new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                            nedPosition, measurements, false,
                            this);
            calibrator.setThreshold(LARGE_THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);
            assertEquals(mCalibrateNextIteration, 0);
            assertEquals(mCalibrateProgressChange, 0);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedMa = calibrator.getEstimatedMa();

            if (!ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMa, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(calibrator.getEstimatedChiSq(), 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateCommonAxisWithInlierNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            CalibrationException, NotReadyException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMaCommonAxis();
            final Matrix mg = generateMg();
            final Matrix gg = generateGg();
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg,
                    OUTLIER_ERROR_FACTOR * accelNoiseRootPSD,
                    OUTLIER_ERROR_FACTOR * gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);
            final IMUErrors errorsInlier = new IMUErrors(ba, bg, ma, mg, gg,
                    accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                    gyroQuantLevel);


            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD()
                    / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD()
                    / sqrtTimeInterval;

            final List<StandardDeviationBodyKinematics> measurements =
                    new ArrayList<>();
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
                final BodyKinematics measuredKinematics;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                    errorsOutlier, random);

                } else {
                    // inlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                    errorsInlier, random);
                }

                final StandardDeviationBodyKinematics measurement =
                        new StandardDeviationBodyKinematics(measuredKinematics,
                                specificForceStandardDeviation,
                                angularRateStandardDeviation);
                measurements.add(measurement);
            }

            final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                    new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(
                            nedPosition, measurements, true,
                            this);
            calibrator.setThreshold(LARGE_THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);
            assertEquals(mCalibrateNextIteration, 0);
            assertEquals(mCalibrateProgressChange, 0);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedMa = calibrator.getEstimatedMa();

            if (!ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ma.equals(estimatedMa, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMa, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(calibrator.getEstimatedChiSq(), 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralNoRefinement() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            CalibrationException, NotReadyException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMaGeneral();
            final Matrix mg = generateMg();
            final Matrix gg = generateGg();
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final IMUErrors errorsOutlier = new IMUErrors(ba, bg, ma, mg, gg,
                    OUTLIER_ERROR_FACTOR * accelNoiseRootPSD,
                    OUTLIER_ERROR_FACTOR * gyroNoiseRootPSD,
                    accelQuantLevel, gyroQuantLevel);
            final IMUErrors errorsInlier = new IMUErrors(ba, bg, ma, mg, gg,
                    0.0, 0.0, accelQuantLevel,
                    gyroQuantLevel);


            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD()
                    / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD()
                    / sqrtTimeInterval;

            final List<StandardDeviationBodyKinematics> measurements =
                    new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
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
                final BodyKinematics measuredKinematics;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                    errorsOutlier, random);

                } else {
                    // inlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                    errorsInlier, random);
                }

                final StandardDeviationBodyKinematics measurement =
                        new StandardDeviationBodyKinematics(measuredKinematics,
                                specificForceStandardDeviation,
                                angularRateStandardDeviation);
                measurements.add(measurement);
            }

            final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator =
                    new MSACRobustKnownBiasAndPositionAccelerometerCalibrator(nedPosition, measurements,
                            false, ba, ma, this);
            calibrator.setThreshold(THRESHOLD);
            calibrator.setResultRefined(false);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);
            assertEquals(mCalibrateNextIteration, 0);
            assertEquals(mCalibrateProgressChange, 0);

            calibrator.calibrate();

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedMa = calibrator.getEstimatedMa();

            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMa, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() >= 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(
            final RobustKnownBiasAndPositionAccelerometerCalibrator calibrator) {
        checkLocked((MSACRobustKnownBiasAndPositionAccelerometerCalibrator) calibrator);
        mCalibrateStart++;
    }

    @Override
    public void onCalibrateEnd(
            final RobustKnownBiasAndPositionAccelerometerCalibrator calibrator) {
        checkLocked((MSACRobustKnownBiasAndPositionAccelerometerCalibrator) calibrator);
        mCalibrateEnd++;
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownBiasAndPositionAccelerometerCalibrator calibrator,
            final int iteration) {
        checkLocked((MSACRobustKnownBiasAndPositionAccelerometerCalibrator) calibrator);
        mCalibrateNextIteration++;
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownBiasAndPositionAccelerometerCalibrator calibrator,
            final float progress) {
        checkLocked((MSACRobustKnownBiasAndPositionAccelerometerCalibrator) calibrator);
        mCalibrateProgressChange++;
    }

    private void reset() {
        mCalibrateStart = 0;
        mCalibrateEnd = 0;
        mCalibrateNextIteration = 0;
        mCalibrateProgressChange = 0;
    }

    private void checkLocked(final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator) {
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
            calibrator.setBias((AccelerationTriad) null);
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
            calibrator.setProgressDelta(0.5f);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setConfidence(0.5);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setMaxIterations(500);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setResultRefined(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setCovarianceKept(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setPreliminarySubsetSize(5);
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
        try {
            calibrator.setThreshold(0.1);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
    }

    private void assertEstimatedResult(
            final Matrix ma,
            final MSACRobustKnownBiasAndPositionAccelerometerCalibrator calibrator) {

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

    private void checkCommonAxisCovariance(final Matrix covariance) {
        assertEquals(covariance.getRows(), 9);
        assertEquals(covariance.getColumns(), 9);

        for (int j = 0; j < 9; j++) {
            final boolean colIsZero = j == 5 || j == 7 || j == 8;
            for (int i = 0; i < 9; i++) {
                final boolean rowIsZero = i == 5 || i == 7 || i == 8;
                if (colIsZero || rowIsZero) {
                    assertEquals(covariance.getElementAt(i, j), 0.0, 0.0);
                }
            }
        }
    }

    private void checkGeneralCovariance(final Matrix covariance) {
        assertEquals(covariance.getRows(), 9);
        assertEquals(covariance.getColumns(), 9);

        for (int i = 0; i < 9; i++) {
            assertNotEquals(covariance.getElementAt(i, i), 0.0);
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
