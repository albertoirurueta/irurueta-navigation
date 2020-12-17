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
import com.irurueta.navigation.inertial.calibration.AccelerationTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
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

public class PROMedSRobustKnownFrameAccelerometerCalibratorTest implements
        RobustKnownFrameAccelerometerCalibratorListener {

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

    private static final int OUTLIER_PERCENTAGE = 20;

    private static final double THRESHOLD = 1e-8;

    private static final double ABSOLUTE_ERROR = 1e-9;

    private static final int TIMES = 100;

    private int mCalibrateStart;
    private int mCalibrateEnd;
    private int mCalibrateNextIteration;
    private int mCalibrateProgressChange;

    @Test
    public void testConstructor() throws WrongSizeException {
        // test empty constructor
        PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                PROMedSRobustKnownFrameAccelerometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        Acceleration acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        Acceleration acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        AccelerationTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        AccelerationTriad initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
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
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        Matrix ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        Matrix ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        Assert.assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS);


        // test constructor with listener
        calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                PROMedSRobustKnownFrameAccelerometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS);


        // test constructor with measurements
        final List<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(measurements);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                PROMedSRobustKnownFrameAccelerometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS);


        // test constructor with measurements and listener
        calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(measurements,
                this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                PROMedSRobustKnownFrameAccelerometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS);


        // test constructor with common axis used
        calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(
                true);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                PROMedSRobustKnownFrameAccelerometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS);


        // test constructor with common axis used and listener
        calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(
                true, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                PROMedSRobustKnownFrameAccelerometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS);


        // test constructor with measurements and common axis used
        calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(
                measurements, true);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                PROMedSRobustKnownFrameAccelerometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS);


        // test constructor with measurements, common axis used and listener
        calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(
                measurements, true, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                PROMedSRobustKnownFrameAccelerometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS);


        // test constructor with quality scores
        final double[] qualityScores = new double[
                RobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS];
        calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(qualityScores);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                PROMedSRobustKnownFrameAccelerometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);


        // test constructor with quality scores and listener
        calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(
                qualityScores, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                PROMedSRobustKnownFrameAccelerometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(
                    new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);


        // test constructor with quality scores and measurements
        calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(qualityScores,
                measurements);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                PROMedSRobustKnownFrameAccelerometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(
                    new double[1], measurements);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);


        // test constructor with quality scores, measurements and listener
        calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(qualityScores,
                measurements, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                PROMedSRobustKnownFrameAccelerometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(
                    new double[1], measurements, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);


        // test constructor with quality scores and common axis used
        calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(qualityScores,
                true);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                PROMedSRobustKnownFrameAccelerometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(
                    new double[1], true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);


        // test constructor with quality scores, common axis used and listener
        calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(qualityScores,
                true, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                PROMedSRobustKnownFrameAccelerometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(
                    new double[1], true, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);


        // test constructor with quality scores, measurements and common axis used
        calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(qualityScores,
                measurements, true);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                PROMedSRobustKnownFrameAccelerometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(
                    new double[1], measurements, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);


        // test constructor with quality scores, measurements, common axis used and
        // listener
        calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(qualityScores,
                measurements, true, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                PROMedSRobustKnownFrameAccelerometerCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROMedS);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        acceleration1 = calibrator.getInitialBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        acceleration1 = calibrator.getInitialBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        acceleration2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(acceleration2);
        assertEquals(acceleration1, acceleration2);
        initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        initialBiasTriad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        ma1 = new Matrix(3, 3);
        assertEquals(calibrator.getInitialMa(), ma1);
        ma2 = new Matrix(3, 3);
        calibrator.getInitialMa(ma2);
        assertEquals(ma1, ma2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameAccelerometerCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(calibrator.getQualityScores(), qualityScores);
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasFx());
        assertNull(calibrator.getEstimatedBiasFy());
        assertNull(calibrator.getEstimatedBiasFz());
        assertNull(calibrator.getEstimatedBiasFxAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertNull(calibrator.getEstimatedBiasFxVariance());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFyVariance());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasFzVariance());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviation());
        assertNull(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(null));
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new PROMedSRobustKnownFrameAccelerometerCalibrator(
                    new double[1], measurements, true, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testGetSetStopThreshold() throws LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default value
        assertEquals(calibrator.getStopThreshold(),
                PROMedSRobustKnownFrameAccelerometerCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);

        // set new value
        calibrator.setStopThreshold(0.1);

        // check
        assertEquals(calibrator.getStopThreshold(), 0.1, 0.0);

        // Force IllegalArgumentException
        try {
            calibrator.setStopThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default value
        assertNull(calibrator.getQualityScores());

        // set new value
        final double[] qualityScores = new double[
                PROMedSRobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS];
        calibrator.setQualityScores(qualityScores);

        // check
        assertSame(calibrator.getQualityScores(), qualityScores);

        // Force IllegalArgumentException
        try {
            calibrator.setQualityScores(new double[3]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetInitialBiasX() throws LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);

        calibrator.setInitialBiasX(biasX);

        // check
        assertEquals(calibrator.getInitialBiasX(), biasX, 0.0);
    }

    @Test
    public void testGetSetInitialBiasY() throws LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double biasY = ba.getElementAtIndex(1);

        calibrator.setInitialBiasY(biasY);

        // check
        assertEquals(calibrator.getInitialBiasY(), biasY, 0.0);
    }

    @Test
    public void testGetSetInitialBiasZ() throws LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double biasZ = ba.getElementAtIndex(2);

        calibrator.setInitialBiasZ(biasZ);

        // check
        assertEquals(calibrator.getInitialBiasZ(), biasZ, 0.0);
    }

    @Test
    public void testGetSetInitialBiasXAsAcceleration() throws LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default value
        final Acceleration biasX1 = calibrator.getInitialBiasXAsAcceleration();

        assertEquals(biasX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(biasX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // set new value
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);

        final Acceleration biasX2 = new Acceleration(biasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setInitialBiasX(biasX2);

        // check
        final Acceleration biasX3 = calibrator.getInitialBiasXAsAcceleration();
        final Acceleration biasX4 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasXAsAcceleration(biasX4);

        assertEquals(biasX2, biasX3);
        assertEquals(biasX2, biasX4);
    }

    @Test
    public void testGetSetInitialBiasYAsAcceleration() throws LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default value
        final Acceleration biasY1 = calibrator.getInitialBiasYAsAcceleration();

        assertEquals(biasY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(biasY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // set new value
        final Matrix ba = generateBa();
        final double biasY = ba.getElementAtIndex(1);

        final Acceleration biasY2 = new Acceleration(biasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setInitialBiasY(biasY2);

        // check
        final Acceleration biasY3 = calibrator.getInitialBiasYAsAcceleration();
        final Acceleration biasY4 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasYAsAcceleration(biasY4);

        assertEquals(biasY2, biasY3);
        assertEquals(biasY2, biasY4);
    }

    @Test
    public void testGetSetInitialBiasZAsAcceleration() throws LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default value
        final Acceleration biasZ1 = calibrator.getInitialBiasZAsAcceleration();

        assertEquals(biasZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(biasZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // set new value
        final Matrix ba = generateBa();
        final double biasZ = ba.getElementAtIndex(2);

        final Acceleration biasZ2 = new Acceleration(biasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setInitialBiasZ(biasZ2);

        // check
        final Acceleration biasZ3 = calibrator.getInitialBiasZAsAcceleration();
        final Acceleration biasZ4 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getInitialBiasZAsAcceleration(biasZ4);

        assertEquals(biasZ2, biasZ3);
        assertEquals(biasZ2, biasZ4);
    }

    @Test
    public void tesSetInitialBias1() throws LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default values
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double biasX = ba.getElementAtIndex(0);
        final double biasY = ba.getElementAtIndex(1);
        final double biasZ = ba.getElementAtIndex(2);

        calibrator.setInitialBias(biasX, biasY, biasZ);

        // check
        assertEquals(biasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(biasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getInitialBiasZ(), 0.0);
    }

    @Test
    public void testSetInitialBias2() throws LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default values
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);

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

        calibrator.setInitialBias(bax, bay, baz);

        // check
        assertEquals(calibrator.getInitialBiasX(), biasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), biasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), biasZ, 0.0);
    }

    @Test
    public void testGetSetInitialBiasAsTriad() {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default values
        final AccelerationTriad triad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(triad1.getValueX(), 0.0, 0.0);
        assertEquals(triad1.getValueY(), 0.0, 0.0);
        assertEquals(triad1.getValueZ(), 0.0, 0.0);
        assertEquals(triad1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final AccelerationTriad triad2 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        // set new values
        final Matrix ba = generateBa();
        final double initialBiasX = ba.getElementAtIndex(0);
        final double initialBiasY = ba.getElementAtIndex(1);
        final double initialBiasZ = ba.getElementAtIndex(2);

        final AccelerationTriad triad3 = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND,
                initialBiasX, initialBiasY, initialBiasZ);
        calibrator.setInitialBias(triad3);

        final AccelerationTriad triad4 = calibrator.getInitialBiasAsTriad();
        final AccelerationTriad triad5 = new AccelerationTriad();
        calibrator.getInitialBiasAsTriad(triad5);

        assertEquals(triad3, triad4);
        assertEquals(triad3, triad5);
    }

    @Test
    public void testGetSetInitialSx() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

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
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

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
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

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
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

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
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

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
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

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
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

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
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

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
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

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
    public void testGetSetInitialScalingFactors() throws WrongSizeException,
            LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

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
    public void testGetSetInitialCrossCouplingErrors() throws WrongSizeException,
            LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

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
    public void testSetInitialScalingFactorsAndCrossCouplingErrors()
            throws WrongSizeException, LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

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
    public void testGetSetInitialBias() throws LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default value
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(bias1, new double[3], 0.0);

        // set new value
        final double[] bias2 = generateBa().getBuffer();
        calibrator.setInitialBias(bias2);

        // check
        final double[] bias3 = calibrator.getInitialBias();
        final double[] bias4 = new double[3];
        calibrator.getInitialBias(bias4);

        assertArrayEquals(bias2, bias3, 0.0);
        assertArrayEquals(bias2, bias4, 0.0);

        // Force IllegalArgumentException
        try {
            calibrator.getInitialBias(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setInitialBias(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetInitialBiasAsMatrix() throws LockedException,
            WrongSizeException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default value
        final Matrix bias1 = calibrator.getInitialBiasAsMatrix();
        assertArrayEquals(bias1.getBuffer(), new double[3], 0.0);

        // set new values
        final Matrix bias2 = generateBa();
        calibrator.setInitialBias(bias2);

        // check
        final Matrix bias3 = calibrator.getInitialBiasAsMatrix();
        final Matrix bias4 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bias4);

        assertEquals(bias2, bias3);
        assertEquals(bias2, bias4);

        // Force IllegalArgumentException
        try {
            calibrator.getInitialBiasAsMatrix(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.getInitialBiasAsMatrix(new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setInitialBias(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setInitialBias(new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetInitialMa() throws WrongSizeException, LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

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
    public void testGetSetMeasurements() throws LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default value
        assertNull(calibrator.getMeasurements());

        // set new value
        final List<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        calibrator.setMeasurements(measurements);

        // check
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testIsSetCommonAxisUsed() throws LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testIsSetLinearCalibratorUsed() throws LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default value
        assertTrue(calibrator.isLinearCalibratorUsed());

        // set new value
        calibrator.setLinearCalibratorUsed(false);

        // check
        assertFalse(calibrator.isLinearCalibratorUsed());
    }

    @Test
    public void testIsSetPreliminarySolutionRefined() throws LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default value
        assertFalse(calibrator.isPreliminarySolutionRefined());

        // set new value
        calibrator.setPreliminarySolutionRefined(true);

        // check
        assertTrue(calibrator.isPreliminarySolutionRefined());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

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
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

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
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

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
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default value
        assertTrue(calibrator.isResultRefined());

        // set new value
        calibrator.setResultRefined(false);

        // check
        assertFalse(calibrator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default value
        assertTrue(calibrator.isCovarianceKept());

        // set new value
        calibrator.setCovarianceKept(false);

        // check
        assertFalse(calibrator.isCovarianceKept());
    }

    @Test
    public void testGetSetPreliminarySubsetSize() throws LockedException {
        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator();

        // check default value
        assertEquals(calibrator.getPreliminarySubsetSize(),
                PROMedSRobustKnownFrameAccelerometerCalibrator.MINIMUM_MEASUREMENTS);

        // set new value
        calibrator.setPreliminarySubsetSize(5);

        // check
        assertEquals(calibrator.getPreliminarySubsetSize(), 5);

        // Force IllegalArgumentException
        try {
            calibrator.setPreliminarySubsetSize(3);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testCalibrateGeneralNoNoiseInlier() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, CalibrationException {
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
                accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);
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

        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                random, 0.0, specificForceStandardDeviation);

        final List<StandardDeviationFrameBodyKinematics> measurements =
                new ArrayList<>();
        final double[] qualityScores = new double[MEASUREMENT_NUMBER];
        double error;
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
                error = Math.abs(errorRandomizer.nextDouble());

            } else {
                // inlier
                measuredKinematics = BodyKinematicsGenerator
                        .generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                errorsInlier, random);
                error = 0.0;
            }

            final StandardDeviationFrameBodyKinematics measurement =
                    new StandardDeviationFrameBodyKinematics(measuredKinematics,
                            ecefFrame, ecefFrame, TIME_INTERVAL_SECONDS,
                            specificForceStandardDeviation,
                            angularRateStandardDeviation);
            measurements.add(measurement);

            qualityScores[i] = 1.0 / (1.0 + error);
        }

        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator(qualityScores,
                        measurements, false, this);
        calibrator.setStopThreshold(THRESHOLD);

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

        final Matrix estimatedBa = calibrator.getEstimatedBiasesAsMatrix();
        final Matrix estimatedMa = calibrator.getEstimatedMa();

        assertTrue(ba.equals(estimatedBa, ABSOLUTE_ERROR));
        assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedBa, estimatedMa, calibrator, true);

        assertNotNull(calibrator.getEstimatedCovariance());
    }

    @Test
    public void testCalibrateCommonAxisNoNoiseInlier() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, CalibrationException {
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
                accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);
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

        final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                random, 0.0, specificForceStandardDeviation);

        final List<StandardDeviationFrameBodyKinematics> measurements =
                new ArrayList<>();
        final double[] qualityScores = new double[MEASUREMENT_NUMBER];
        double error;
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
                error = Math.abs(errorRandomizer.nextDouble());

            } else {
                // inlier
                measuredKinematics = BodyKinematicsGenerator
                        .generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                errorsInlier, random);
                error = 0.0;
            }

            final StandardDeviationFrameBodyKinematics measurement =
                    new StandardDeviationFrameBodyKinematics(measuredKinematics,
                            ecefFrame, ecefFrame, TIME_INTERVAL_SECONDS,
                            specificForceStandardDeviation,
                            angularRateStandardDeviation);
            measurements.add(measurement);

            qualityScores[i] = 1.0 / (1.0 + error);
        }

        final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                new PROMedSRobustKnownFrameAccelerometerCalibrator(qualityScores,
                        measurements, true, this);
        calibrator.setStopThreshold(THRESHOLD);

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

        final Matrix estimatedBa = calibrator.getEstimatedBiasesAsMatrix();
        final Matrix estimatedMa = calibrator.getEstimatedMa();

        assertTrue(ba.equals(estimatedBa, ABSOLUTE_ERROR));
        assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedBa, estimatedMa, calibrator, true);

        assertNotNull(calibrator.getEstimatedCovariance());
    }

    @Test
    public void testCalibrateGeneralNoRefinement() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, CalibrationException {

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
                    accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);
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

            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    random, 0.0, specificForceStandardDeviation);

            final List<StandardDeviationFrameBodyKinematics> measurements =
                    new ArrayList<>();
            final double[] qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
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
                    error = Math.abs(errorRandomizer.nextDouble());

                } else {
                    // inlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                    errorsInlier, random);
                    error = 0.0;
                }

                final StandardDeviationFrameBodyKinematics measurement =
                        new StandardDeviationFrameBodyKinematics(measuredKinematics,
                                ecefFrame, ecefFrame, TIME_INTERVAL_SECONDS,
                                specificForceStandardDeviation,
                                angularRateStandardDeviation);
                measurements.add(measurement);

                qualityScores[i] = 1.0 / (1.0 + error);
            }

            final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                    new PROMedSRobustKnownFrameAccelerometerCalibrator(qualityScores,
                            measurements, false, this);
            calibrator.setStopThreshold(THRESHOLD);
            calibrator.setResultRefined(false);
            calibrator.setPreliminarySolutionRefined(false);

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

            assertEstimatedResult(estimatedBa, estimatedMa, calibrator, false);

            assertNull(calibrator.getEstimatedCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralNonLinearWithInitialValue()
            throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, CalibrationException {

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
                    accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);
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

            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    random, 0.0, specificForceStandardDeviation);

            final List<StandardDeviationFrameBodyKinematics> measurements =
                    new ArrayList<>();
            final double[] qualityScores = new double[MEASUREMENT_NUMBER];
            double error;
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
                    error = Math.abs(errorRandomizer.nextDouble());

                } else {
                    // inlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(TIME_INTERVAL_SECONDS, trueKinematics,
                                    errorsInlier, random);
                    error = 0.0;
                }

                final StandardDeviationFrameBodyKinematics measurement =
                        new StandardDeviationFrameBodyKinematics(measuredKinematics,
                                ecefFrame, ecefFrame, TIME_INTERVAL_SECONDS,
                                specificForceStandardDeviation,
                                angularRateStandardDeviation);
                measurements.add(measurement);

                qualityScores[i] = 1.0 / (1.0 + error);
            }

            final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator =
                    new PROMedSRobustKnownFrameAccelerometerCalibrator(qualityScores,
                            measurements, false, this);
            calibrator.setStopThreshold(THRESHOLD);
            calibrator.setInitialBias(ba);
            calibrator.setInitialMa(ma);
            calibrator.setLinearCalibratorUsed(false);
            calibrator.setPreliminarySolutionRefined(true);

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

            assertEstimatedResult(estimatedBa, estimatedMa, calibrator, true);

            assertNotNull(calibrator.getEstimatedCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(RobustKnownFrameAccelerometerCalibrator calibrator) {
        checkLocked((PROMedSRobustKnownFrameAccelerometerCalibrator) calibrator);
        mCalibrateStart++;
    }

    @Override
    public void onCalibrateEnd(RobustKnownFrameAccelerometerCalibrator calibrator) {
        checkLocked((PROMedSRobustKnownFrameAccelerometerCalibrator) calibrator);
        mCalibrateEnd++;
    }

    @Override
    public void onCalibrateNextIteration(RobustKnownFrameAccelerometerCalibrator calibrator, int iteration) {
        checkLocked((PROMedSRobustKnownFrameAccelerometerCalibrator) calibrator);
        mCalibrateNextIteration++;
    }

    @Override
    public void onCalibrateProgressChange(RobustKnownFrameAccelerometerCalibrator calibrator, float progress) {
        checkLocked((PROMedSRobustKnownFrameAccelerometerCalibrator) calibrator);
        mCalibrateProgressChange++;
    }

    private void reset() {
        mCalibrateStart = 0;
        mCalibrateEnd = 0;
        mCalibrateNextIteration = 0;
        mCalibrateProgressChange = 0;
    }

    private void checkLocked(final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator) {
        try {
            calibrator.setInitialBiasX(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialBiasY(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialBiasZ(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialBiasX(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialBiasY(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialBiasZ(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialBias(0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialBias(null, null, null);
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
            calibrator.setInitialBias((double[]) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialBias((Matrix) null);
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
            calibrator.setLinearCalibratorUsed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setPreliminarySolutionRefined(true);
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
            calibrator.setStopThreshold(0.1);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setQualityScores(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
    }

    private void assertEstimatedResult(
            final Matrix ba, final Matrix ma,
            final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator,
            final boolean checkCovariance)
            throws WrongSizeException {

        final double[] estimatedBiases = calibrator.getEstimatedBiases();
        assertArrayEquals(ba.getBuffer(), estimatedBiases, 0.0);

        final double[] estimatedBiases2 = new double[3];
        calibrator.getEstimatedBiases(estimatedBiases2);
        assertArrayEquals(estimatedBiases, estimatedBiases2, 0.0);

        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getEstimatedBiasesAsMatrix(ba2);

        assertEquals(ba, ba2);

        assertEquals(ba.getElementAtIndex(0), calibrator.getEstimatedBiasFx(),
                0.0);
        assertEquals(ba.getElementAtIndex(1), calibrator.getEstimatedBiasFy(),
                0.0);
        assertEquals(ba.getElementAtIndex(2), calibrator.getEstimatedBiasFz(),
                0.0);

        final Acceleration bax1 = calibrator.getEstimatedBiasFxAsAcceleration();
        final Acceleration bax2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getEstimatedBiasFxAsAcceleration(bax2);
        assertEquals(bax1, bax2);
        assertEquals(calibrator.getEstimatedBiasFx(),
                bax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());

        final Acceleration bay1 = calibrator.getEstimatedBiasFyAsAcceleration();
        final Acceleration bay2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getEstimatedBiasFyAsAcceleration(bay2);
        assertEquals(bay1, bay2);
        assertEquals(calibrator.getEstimatedBiasFy(),
                bay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());

        final Acceleration baz1 = calibrator.getEstimatedBiasFzAsAcceleration();
        final Acceleration baz2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getEstimatedBiasFzAsAcceleration(baz2);
        assertEquals(baz1, baz2);
        assertEquals(calibrator.getEstimatedBiasFz(),
                baz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());

        final AccelerationTriad bTriad1 = calibrator.getEstimatedBiasAsTriad();
        assertEquals(calibrator.getEstimatedBiasFx(), bTriad1.getValueX(), 0.0);
        assertEquals(calibrator.getEstimatedBiasFy(), bTriad1.getValueY(), 0.0);
        assertEquals(calibrator.getEstimatedBiasFz(), bTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bTriad1.getUnit());
        final AccelerationTriad bTriad2 = new AccelerationTriad();
        calibrator.getEstimatedBiasAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);

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

        if (checkCovariance) {
            assertCovariance(calibrator);
        }
    }

    private void assertCovariance(
            final PROMedSRobustKnownFrameAccelerometerCalibrator calibrator) {
        assertNotNull(calibrator.getEstimatedBiasFxVariance());

        assertNotNull(calibrator.getEstimatedBiasFxVariance());
        assertNotNull(calibrator.getEstimatedBiasFxStandardDeviation());
        final Acceleration stdBx1 = calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration();
        assertNotNull(stdBx1);
        final Acceleration stdBx2 = new Acceleration(1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertTrue(calibrator.getEstimatedBiasFxStandardDeviationAsAcceleration(stdBx2));
        assertEquals(stdBx1, stdBx2);

        assertNotNull(calibrator.getEstimatedBiasFyVariance());
        assertNotNull(calibrator.getEstimatedBiasFyStandardDeviation());
        final Acceleration stdBy1 = calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration();
        assertNotNull(stdBy1);
        final Acceleration stdBy2 = new Acceleration(1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertTrue(calibrator.getEstimatedBiasFyStandardDeviationAsAcceleration(stdBy2));
        assertEquals(stdBy1, stdBy2);

        assertNotNull(calibrator.getEstimatedBiasFzVariance());
        assertNotNull(calibrator.getEstimatedBiasFzStandardDeviation());
        final Acceleration stdBz1 = calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration();
        assertNotNull(stdBz1);
        final Acceleration stdBz2 = new Acceleration(1.0, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        assertTrue(calibrator.getEstimatedBiasFzStandardDeviationAsAcceleration(stdBz2));
        assertEquals(stdBz1, stdBz2);

        final AccelerationTriad std1 = calibrator.getEstimatedBiasStandardDeviation();
        assertEquals(calibrator.getEstimatedBiasFxStandardDeviation(), std1.getValueX(), 0.0);
        assertEquals(calibrator.getEstimatedBiasFyStandardDeviation(), std1.getValueY(), 0.0);
        assertEquals(calibrator.getEstimatedBiasFzStandardDeviation(), std1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, std1.getUnit());
        final AccelerationTriad std2 = new AccelerationTriad();
        calibrator.getEstimatedBiasStandardDeviation(std2);

        final double avgStd = (calibrator.getEstimatedBiasFxStandardDeviation() +
                calibrator.getEstimatedBiasFyStandardDeviation() +
                calibrator.getEstimatedBiasFzStandardDeviation()) / 3.0;
        assertEquals(avgStd, calibrator.getEstimatedBiasStandardDeviationAverage(), 0.0);
        final Acceleration avg1 = calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration();
        assertEquals(avgStd, avg1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, avg1.getUnit());
        final Acceleration avg2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getEstimatedBiasStandardDeviationAverageAsAcceleration(avg2);
        assertEquals(avg1, avg2);

        assertEquals(std1.getNorm(), calibrator.getEstimatedBiasStandardDeviationNorm(), ABSOLUTE_ERROR);
        final Acceleration norm1 = calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration();
        assertEquals(std1.getNorm(), norm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, norm1.getUnit());
        final Acceleration norm2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getEstimatedBiasStandardDeviationNormAsAcceleration(norm2);
        assertEquals(norm1, norm2);
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
