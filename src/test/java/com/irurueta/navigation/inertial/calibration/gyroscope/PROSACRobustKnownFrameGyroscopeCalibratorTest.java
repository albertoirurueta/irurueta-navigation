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
package com.irurueta.navigation.inertial.calibration.gyroscope;

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
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import org.junit.Assert;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class PROSACRobustKnownFrameGyroscopeCalibratorTest
        implements RobustKnownFrameGyroscopeCalibratorListener {

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

    private static final double THRESHOLD = 1e-18;
    private static final double LARGE_THRESHOLD = 5e-4;

    private static final double LARGE_ABSOLUTE_ERROR = 5e-5;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;

    private static final double OUTLIER_ERROR_FACTOR = 100.0;

    private static final int TIMES = 100;

    private int mCalibrateStart;
    private int mCalibrateEnd;
    private int mCalibrateNextIteration;
    private int mCalibrateProgressChange;

    @Test
    public void testConstructor1() throws WrongSizeException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getThreshold(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        final AngularSpeed bgx1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);
        final AngularSpeed bgy1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgy1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);
        final AngularSpeed bgz1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);
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
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, new Matrix(3, 3));
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, new Matrix(3, 3));
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        Assert.assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor2() throws WrongSizeException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator(this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        final AngularSpeed bgx1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);
        final AngularSpeed bgy1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgy1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);
        final AngularSpeed bgz1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);
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
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, new Matrix(3, 3));
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, new Matrix(3, 3));
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor3() throws WrongSizeException {
        final List<StandardDeviationFrameBodyKinematics> measurements = Collections.emptyList();

        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator(measurements);

        // check default values
        assertEquals(calibrator.getThreshold(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        final AngularSpeed bgx1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);
        final AngularSpeed bgy1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgy1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);
        final AngularSpeed bgz1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);
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
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, new Matrix(3, 3));
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, new Matrix(3, 3));
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor4() throws WrongSizeException {
        final List<StandardDeviationFrameBodyKinematics> measurements = Collections.emptyList();

        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator(measurements, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        final AngularSpeed bgx1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);
        final AngularSpeed bgy1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgy1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);
        final AngularSpeed bgz1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);
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
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, new Matrix(3, 3));
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, new Matrix(3, 3));
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor5() throws WrongSizeException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator(true);

        // check default values
        assertEquals(calibrator.getThreshold(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        final AngularSpeed bgx1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);
        final AngularSpeed bgy1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgy1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);
        final AngularSpeed bgz1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);
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
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, new Matrix(3, 3));
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, new Matrix(3, 3));
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor6() throws WrongSizeException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator(true, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        final AngularSpeed bgx1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);
        final AngularSpeed bgy1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgy1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);
        final AngularSpeed bgz1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);
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
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, new Matrix(3, 3));
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, new Matrix(3, 3));
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor7() throws WrongSizeException {
        final List<StandardDeviationFrameBodyKinematics> measurements = Collections.emptyList();

        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator(measurements, true);

        // check default values
        assertEquals(calibrator.getThreshold(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        final AngularSpeed bgx1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);
        final AngularSpeed bgy1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgy1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);
        final AngularSpeed bgz1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);
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
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, new Matrix(3, 3));
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, new Matrix(3, 3));
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor8() throws WrongSizeException {
        final List<StandardDeviationFrameBodyKinematics> measurements = Collections.emptyList();

        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator(measurements, true, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        final AngularSpeed bgx1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);
        final AngularSpeed bgy1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgy1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);
        final AngularSpeed bgz1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);
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
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, new Matrix(3, 3));
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, new Matrix(3, 3));
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor9() throws WrongSizeException {
        final double[] qualityScores = new double[
                PROSACRobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS];
        PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores);

        // check default values
        assertEquals(calibrator.getThreshold(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        final AngularSpeed bgx1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);
        final AngularSpeed bgy1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgy1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);
        final AngularSpeed bgz1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);
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
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, new Matrix(3, 3));
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, new Matrix(3, 3));
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new PROSACRobustKnownFrameGyroscopeCalibrator(
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor10() throws WrongSizeException {
        final double[] qualityScores = new double[
                PROSACRobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS];
        PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores,
                        this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        final AngularSpeed bgx1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);
        final AngularSpeed bgy1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgy1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);
        final AngularSpeed bgz1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);
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
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, new Matrix(3, 3));
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, new Matrix(3, 3));
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new PROSACRobustKnownFrameGyroscopeCalibrator(
                    new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor11() throws WrongSizeException {
        final double[] qualityScores = new double[
                PROSACRobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS];
        final List<StandardDeviationFrameBodyKinematics> measurements = Collections.emptyList();

        PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores,
                        measurements);

        // check default values
        assertEquals(calibrator.getThreshold(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        final AngularSpeed bgx1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);
        final AngularSpeed bgy1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgy1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);
        final AngularSpeed bgz1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);
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
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, new Matrix(3, 3));
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, new Matrix(3, 3));
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new PROSACRobustKnownFrameGyroscopeCalibrator(
                    new double[1], measurements);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor12() throws WrongSizeException {
        final double[] qualityScores = new double[
                PROSACRobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS];
        final List<StandardDeviationFrameBodyKinematics> measurements = Collections.emptyList();

        PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores,
                        measurements, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        final AngularSpeed bgx1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);
        final AngularSpeed bgy1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgy1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);
        final AngularSpeed bgz1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);
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
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, new Matrix(3, 3));
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, new Matrix(3, 3));
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new PROSACRobustKnownFrameGyroscopeCalibrator(new double[1],
                    measurements, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor13() throws WrongSizeException{
        final double[] qualityScores = new double[
                PROSACRobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS];
        PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator(
                        qualityScores, true);

        // check default values
        assertEquals(calibrator.getThreshold(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        final AngularSpeed bgx1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);
        final AngularSpeed bgy1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgy1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);
        final AngularSpeed bgz1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);
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
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, new Matrix(3, 3));
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, new Matrix(3, 3));
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new PROSACRobustKnownFrameGyroscopeCalibrator(
                    new double[1], true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor14() throws WrongSizeException {
        final double[] qualityScores = new double[
                PROSACRobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS];

        PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores,
                        true, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        final AngularSpeed bgx1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);
        final AngularSpeed bgy1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgy1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);
        final AngularSpeed bgz1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);
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
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, new Matrix(3, 3));
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, new Matrix(3, 3));
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new PROSACRobustKnownFrameGyroscopeCalibrator(
                    new double[1], true, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor15() throws WrongSizeException {
        final double[] qualityScores = new double[
                PROSACRobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS];
        final List<StandardDeviationFrameBodyKinematics> measurements = Collections.emptyList();

        PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores,
                        measurements, true);

        // check default values
        assertEquals(calibrator.getThreshold(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        final AngularSpeed bgx1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);
        final AngularSpeed bgy1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgy1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);
        final AngularSpeed bgz1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);
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
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, new Matrix(3, 3));
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, new Matrix(3, 3));
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new PROSACRobustKnownFrameGyroscopeCalibrator(
                    new double[1], measurements, true);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor16() throws WrongSizeException {
        final double[] qualityScores = new double[
                PROSACRobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS];
        final List<StandardDeviationFrameBodyKinematics> measurements = Collections.emptyList();

        PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores,
                        measurements, true, this);

        // check default values
        assertEquals(calibrator.getThreshold(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_THRESHOLD, 0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.PROSAC);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        final AngularSpeed bgx1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);
        final AngularSpeed bgy1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgy1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);
        final AngularSpeed bgz1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);
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
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = new Matrix(3, 1);
        assertEquals(calibrator.getInitialBiasAsMatrix(), biasMatrix1);
        final Matrix biasMatrix2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, new Matrix(3, 3));
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, new Matrix(3, 3));
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS, 0.0);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertSame(qualityScores, calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedBiases());
        assertFalse(calibrator.getEstimatedBiases(null));
        assertNull(calibrator.getEstimatedBiasesAsMatrix());
        assertFalse(calibrator.getEstimatedBiasesAsMatrix(null));
        assertNull(calibrator.getEstimatedBiasX());
        assertNull(calibrator.getEstimatedBiasY());
        assertNull(calibrator.getEstimatedBiasZ());
        assertNull(calibrator.getEstimatedBiasAngularSpeedX());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedX(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedY());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedY(null));
        assertNull(calibrator.getEstimatedBiasAngularSpeedZ());
        assertFalse(calibrator.getEstimatedBiasAngularSpeedZ(null));
        assertNull(calibrator.getEstimatedMg());
        assertNull(calibrator.getEstimatedSx());
        assertNull(calibrator.getEstimatedSy());
        assertNull(calibrator.getEstimatedSz());
        assertNull(calibrator.getEstimatedMxy());
        assertNull(calibrator.getEstimatedMxz());
        assertNull(calibrator.getEstimatedMyx());
        assertNull(calibrator.getEstimatedMyz());
        assertNull(calibrator.getEstimatedMzx());
        assertNull(calibrator.getEstimatedMzy());
        assertNull(calibrator.getEstimatedGg());
        assertNull(calibrator.getEstimatedCovariance());
        assertEquals(calibrator.getPreliminarySubsetSize(),
                RobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new PROSACRobustKnownFrameGyroscopeCalibrator(
                    new double[1], measurements, true, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getThreshold(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_THRESHOLD, 0.0);

        // set new value
        calibrator.setThreshold(1.0);

        // check
        assertEquals(calibrator.getThreshold(), 1.0, 0.0);

        // Force IllegalArgumentException
        try {
            calibrator.setThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetComputeAndKeepInliersEnabled() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertFalse(calibrator.isComputeAndKeepInliersEnabled());

        // set new value
        calibrator.setComputeAndKeepInliersEnabled(true);

        // check
        assertTrue(calibrator.isComputeAndKeepInliersEnabled());
    }

    @Test
    public void testIsSetComputeAndKeepResidualsEnabled() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertFalse(calibrator.isComputeAndKeepResiduals());

        // set new value
        calibrator.setComputeAndKeepResidualsEnabled(true);

        // check
        assertTrue(calibrator.isComputeAndKeepResiduals());
    }

    @Test
    public void testGetSetInitialBiasX() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double biasX = bg.getElementAtIndex(0);

        calibrator.setInitialBiasX(biasX);

        // check
        assertEquals(calibrator.getInitialBiasX(), biasX, 0.0);
    }

    @Test
    public void testGetSetInitialBiasY() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double biasY = bg.getElementAtIndex(1);

        calibrator.setInitialBiasY(biasY);

        // check
        assertEquals(calibrator.getInitialBiasY(), biasY, 0.0);
    }

    @Test
    public void testGetSetInitialBiasZ() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double biasZ = bg.getElementAtIndex(2);

        calibrator.setInitialBiasZ(biasZ);

        // check
        assertEquals(calibrator.getInitialBiasZ(), biasZ, 0.0);
    }

    @Test
    public void testGetSetInitialBiasAngularSpeedX() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        final AngularSpeed bx1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

        // set new value
        final Matrix bg = generateBg();
        final double biasX = bg.getElementAtIndex(0);
        final AngularSpeed bx2 = new AngularSpeed(biasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setInitialBiasX(bx2);

        // check
        final AngularSpeed bx3 = calibrator.getInitialBiasAngularSpeedX();
        final AngularSpeed bx4 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bx4);

        assertEquals(bx2, bx3);
        assertEquals(bx2, bx4);
    }

    @Test
    public void testGetSetInitialBiasAngularSpeedY() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        final AngularSpeed by1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

        // set new value
        final Matrix bg = generateBg();
        final double biasY = bg.getElementAtIndex(1);
        final AngularSpeed by2 = new AngularSpeed(biasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setInitialBiasY(by2);

        // check
        final AngularSpeed by3 = calibrator.getInitialBiasAngularSpeedY();
        final AngularSpeed by4 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(by4);

        assertEquals(by2, by3);
        assertEquals(by2, by4);
    }

    @Test
    public void testGetSetInitialBiasAngularSpeedZ() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        final AngularSpeed bz1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

        // set new value
        final Matrix bg = generateBg();
        final double biasZ = bg.getElementAtIndex(2);
        final AngularSpeed bz2 = new AngularSpeed(biasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setInitialBiasZ(bz2);

        // check
        final AngularSpeed bz3 = calibrator.getInitialBiasAngularSpeedZ();
        final AngularSpeed bz4 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bz4);

        assertEquals(bz2, bz3);
        assertEquals(bz2, bz4);
    }

    @Test
    public void testSetInitialBias1() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check initial values
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);

        // set new values
        final Matrix bg = generateBg();
        final double biasX = bg.getElementAtIndex(0);
        final double biasY = bg.getElementAtIndex(1);
        final double biasZ = bg.getElementAtIndex(2);

        calibrator.setInitialBias(biasX, biasY, biasZ);

        // check
        assertEquals(biasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(biasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getInitialBiasZ(), 0.0);
    }

    @Test
    public void testSetInitialBias2() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check initial values
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);

        // set new values
        final Matrix bg = generateBg();
        final double biasX = bg.getElementAtIndex(0);
        final double biasY = bg.getElementAtIndex(1);
        final double biasZ = bg.getElementAtIndex(2);

        final AngularSpeed bx = new AngularSpeed(biasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by = new AngularSpeed(biasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bz = new AngularSpeed(biasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setInitialBias(bx, by, bz);

        // check
        assertEquals(biasX, calibrator.getInitialBiasX(), 0.0);
        assertEquals(biasY, calibrator.getInitialBiasY(), 0.0);
        assertEquals(biasZ, calibrator.getInitialBiasZ(), 0.0);
    }

    @Test
    public void testGetSetInitialSx() throws WrongSizeException, LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateMg();
        final double initialSx = mg.getElementAt(0, 0);

        calibrator.setInitialSx(initialSx);

        // check
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
    }

    @Test
    public void testGetSetInitialSy() throws WrongSizeException, LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateMg();
        final double initialSy = mg.getElementAt(1, 1);

        calibrator.setInitialSy(initialSy);

        // check
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
    }

    @Test
    public void testGetSetInitialSz() throws WrongSizeException, LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateMg();
        final double initialSz = mg.getElementAt(2, 2);

        calibrator.setInitialSz(initialSz);

        // check
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
    }

    @Test
    public void testGetSetInitialMxy() throws WrongSizeException, LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateMg();
        final double initialMxy = mg.getElementAt(0, 1);

        calibrator.setInitialMxy(initialMxy);

        // check
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
    }

    @Test
    public void testGetSetInitialMxz() throws WrongSizeException, LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateMg();
        final double initialMxz = mg.getElementAt(0, 2);

        calibrator.setInitialMxz(initialMxz);

        // check
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
    }

    @Test
    public void testGetSetInitialMyx() throws WrongSizeException, LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateMg();
        final double initialMyx = mg.getElementAt(1, 0);

        calibrator.setInitialMyx(initialMyx);

        // check
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
    }

    @Test
    public void testGetSetInitialMyz() throws WrongSizeException, LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateMg();
        final double initialMyz = mg.getElementAt(1, 2);

        calibrator.setInitialMyz(initialMyz);

        // check
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
    }

    @Test
    public void testGetSetInitialMzx() throws WrongSizeException, LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateMg();
        final double initialMzx = mg.getElementAt(2, 0);

        calibrator.setInitialMzx(initialMzx);

        // check
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
    }

    @Test
    public void testGetSetInitialMzy() throws WrongSizeException, LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateMg();
        final double initialMzy = mg.getElementAt(2, 1);

        calibrator.setInitialMzy(initialMzy);

        // check
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
    }

    @Test
    public void testSetInitialScalingFactors() throws WrongSizeException, LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateMg();
        final double initialSx = mg.getElementAt(0, 0);
        final double initialSy = mg.getElementAt(1, 1);
        final double initialSz = mg.getElementAt(2, 2);

        calibrator.setInitialScalingFactors(initialSx, initialSy, initialSz);

        // check
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
    }

    @Test
    public void testSetInitialCrossCouplingErrors() throws WrongSizeException, LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        // set new values
        final Matrix mg = generateMg();
        final double initialMxy = mg.getElementAt(0, 1);
        final double initialMxz = mg.getElementAt(0, 2);
        final double initialMyx = mg.getElementAt(1, 0);
        final double initialMyz = mg.getElementAt(1, 2);
        final double initialMzx = mg.getElementAt(2, 0);
        final double initialMzy = mg.getElementAt(2, 1);

        calibrator.setInitialCrossCouplingErrors(initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);

        // check
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
    }

    @Test
    public void testSetInitialScalingFactorsAndCrossCouplingErrors() throws WrongSizeException, LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

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
        final Matrix mg = generateMg();
        final double initialSx = mg.getElementAt(0, 0);
        final double initialSy = mg.getElementAt(1, 1);
        final double initialSz = mg.getElementAt(2, 2);
        final double initialMxy = mg.getElementAt(0, 1);
        final double initialMxz = mg.getElementAt(0, 2);
        final double initialMyx = mg.getElementAt(1, 0);
        final double initialMyz = mg.getElementAt(1, 2);
        final double initialMzx = mg.getElementAt(2, 0);
        final double initialMzy = mg.getElementAt(2, 1);

        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
                initialSx, initialSy, initialSz, initialMxy, initialMxz, initialMyx,
                initialMyz, initialMzx, initialMzy);

        // check
        assertEquals(initialSx, calibrator.getInitialSx(), 0.0);
        assertEquals(initialSy, calibrator.getInitialSy(), 0.0);
        assertEquals(initialSz, calibrator.getInitialSz(), 0.0);
        assertEquals(initialMxy, calibrator.getInitialMxy(), 0.0);
        assertEquals(initialMxz, calibrator.getInitialMxz(), 0.0);
        assertEquals(initialMyx, calibrator.getInitialMyx(), 0.0);
        assertEquals(initialMyz, calibrator.getInitialMyz(), 0.0);
        assertEquals(initialMzx, calibrator.getInitialMzx(), 0.0);
        assertEquals(initialMzy, calibrator.getInitialMzy(), 0.0);
    }

    @Test
    public void testGetSetInitialBias() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        final double[] bias1 = calibrator.getInitialBias();
        assertArrayEquals(new double[3], bias1, 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double[] bias2 = bg.getBuffer();

        calibrator.setInitialBias(bias2);

        // check
        final double[] bias3 = calibrator.getInitialBias();
        final double[] bias4 = new double[3];
        calibrator.getInitialBias(bias4);

        assertArrayEquals(bias2, bias3, 0.0);
        assertArrayEquals(bias2, bias4, 0.0);

        // Force IllegalArgumentException
        try {
            calibrator.setInitialBias(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetInitialBiasAsMatrix() throws WrongSizeException, LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        final Matrix bias1 = calibrator.getInitialBiasAsMatrix();

        assertEquals(bias1, new Matrix(3, 1));

        // set new value
        final Matrix bias2 = generateBg();

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
    public void testGetSetInitialMg() throws WrongSizeException, LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(new Matrix(3, 3), mg1);

        // set new value
        final Matrix mg2 = generateMg();
        calibrator.setInitialMg(mg2);

        // check
        final Matrix mg3 = calibrator.getInitialMg();
        final Matrix mg4 = new Matrix(3, 3);
        calibrator.getInitialMg(mg4);

        assertEquals(mg2, mg3);
        assertEquals(mg2, mg4);

        // Force IllegalArgumentException
        try {
            calibrator.getInitialMg(new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.getInitialMg(new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setInitialMg(new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setInitialMg(new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetInitialGg() throws WrongSizeException, LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, new Matrix(3, 3));

        // set new value
        final Matrix gg2 = generateGg();
        calibrator.setInitialGg(gg2);

        // check
        final Matrix gg3 = calibrator.getInitialGg();
        final Matrix gg4 = new Matrix(3, 3);
        calibrator.getInitialGg(gg4);

        assertEquals(gg2, gg3);
        assertEquals(gg2, gg4);

        // Force IllegalArgumentException
        try {
            calibrator.getInitialGg(new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.getInitialGg(new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setInitialGg(new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setInitialGg(new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMeasurements() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertNull(calibrator.getMeasurements());

        // set new value
        final List<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();

        calibrator.setMeasurements(measurements);

        // check
        assertSame(measurements, calibrator.getMeasurements());
    }

    @Test
    public void testIsSetCommonAxisUsed() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testIsSetLinearCalibratorUsed() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertTrue(calibrator.isLinearCalibratorUsed());

        // set new value
        calibrator.setLinearCalibratorUsed(false);

        // check
        assertFalse(calibrator.isLinearCalibratorUsed());
    }

    @Test
    public void testIsSetPreliminarySolutionRefined() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertFalse(calibrator.isPreliminarySolutionRefined());

        // set new value
        calibrator.setPreliminarySolutionRefined(true);

        // check
        assertTrue(calibrator.isPreliminarySolutionRefined());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getProgressDelta(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);

        // set new value
        calibrator.setProgressDelta(0.5f);

        // check
        assertEquals(calibrator.getProgressDelta(), 0.5f, 0.0);

        // Force IllegalArgumentException
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
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getConfidence(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);

        // set new value
        calibrator.setConfidence(0.8);

        // check
        assertEquals(calibrator.getConfidence(), 0.8, 0.0);

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
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getMaxIterations(),
                PROSACRobustKnownFrameGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);

        // set new value
        calibrator.setMaxIterations(1);

        assertEquals(calibrator.getMaxIterations(), 1);

        // force IllegalArgumentException
        try {
            calibrator.setMaxIterations(0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetResultRefined() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertTrue(calibrator.isResultRefined());

        // set new value
        calibrator.setResultRefined(false);

        // check
        assertFalse(calibrator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertTrue(calibrator.isCovarianceKept());

        // set new value
        calibrator.setCovarianceKept(false);

        // check
        assertFalse(calibrator.isCovarianceKept());
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertNull(calibrator.getQualityScores());

        // set new value
        final double[] qualityScores = new double[
                PROSACRobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS];
        calibrator.setQualityScores(qualityScores);

        // check
        assertSame(qualityScores, calibrator.getQualityScores());

        // Force IllegalArgumentException
        try {
            calibrator.setQualityScores(new double[6]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetPreliminarySubsetSize() throws LockedException {
        final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                new PROSACRobustKnownFrameGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getPreliminarySubsetSize(),
                PROSACRobustKnownFrameGyroscopeCalibrator.MINIMUM_MEASUREMENTS);

        // set new value
        calibrator.setPreliminarySubsetSize(8);

        // check
        assertEquals(calibrator.getPreliminarySubsetSize(), 8);

        // force IllegalArgumentException
        try {
            calibrator.setPreliminarySubsetSize(6);
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
                    random, 0.0, angularRateStandardDeviation);

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

            final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                    new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores,
                            measurements, false, this);
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

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertNotNull(estimatedMg);
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());

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
                    random, 0.0, angularRateStandardDeviation);

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

            final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                    new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores,
                            measurements, true, this);
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

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());

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
                    OUTLIER_ERROR_FACTOR * gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);
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

            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    random, 0.0, angularRateStandardDeviation);

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

            final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                    new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores,
                            measurements, false, this);
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

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertNotNull(estimatedMg);
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());

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
                    OUTLIER_ERROR_FACTOR * gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);
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

            final GaussianRandomizer errorRandomizer = new GaussianRandomizer(
                    random, 0.0, angularRateStandardDeviation);

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

            final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                    new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores,
                            measurements, true, this);
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

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertNotNull(estimatedMg);
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());

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
                    random, 0.0, angularRateStandardDeviation);

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

            final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                    new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores,
                            measurements, false, this);
            calibrator.setThreshold(THRESHOLD);
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

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertNotNull(estimatedMg);
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNull(calibrator.getEstimatedCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralNonLinearWithInitialValue() throws WrongSizeException,
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
                    random, 0.0, angularRateStandardDeviation);

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

            final PROSACRobustKnownFrameGyroscopeCalibrator calibrator =
                    new PROSACRobustKnownFrameGyroscopeCalibrator(qualityScores,
                            measurements, false, this);
            calibrator.setThreshold(THRESHOLD);
            calibrator.setInitialBias(bg);
            calibrator.setInitialMg(mg);
            calibrator.setInitialGg(gg);
            calibrator.setLinearCalibratorUsed(false);

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

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertNotNull(estimatedMg);
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(final RobustKnownFrameGyroscopeCalibrator calibrator) {
        checkLocked((PROSACRobustKnownFrameGyroscopeCalibrator) calibrator);
        mCalibrateStart++;
    }

    @Override
    public void onCalibrateEnd(final RobustKnownFrameGyroscopeCalibrator calibrator) {
        checkLocked((PROSACRobustKnownFrameGyroscopeCalibrator) calibrator);
        mCalibrateEnd++;
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownFrameGyroscopeCalibrator calibrator, final int iteration) {
        checkLocked((PROSACRobustKnownFrameGyroscopeCalibrator) calibrator);
        mCalibrateNextIteration++;
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownFrameGyroscopeCalibrator calibrator, final float progress) {
        checkLocked((PROSACRobustKnownFrameGyroscopeCalibrator) calibrator);
        mCalibrateProgressChange++;
    }

    private void reset() {
        mCalibrateStart = 0;
        mCalibrateEnd = 0;
        mCalibrateNextIteration = 0;
        mCalibrateProgressChange = 0;
    }

    private void checkLocked(final PROSACRobustKnownFrameGyroscopeCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
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
            calibrator.setInitialMg(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialGg(null);
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
            calibrator.setThreshold(0.1);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setQualityScores(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setComputeAndKeepInliersEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setComputeAndKeepResidualsEnabled(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
    }

    private void assertEstimatedResult(
            final Matrix bg, final Matrix mg, final Matrix gg,
            final PROSACRobustKnownFrameGyroscopeCalibrator calibrator)
            throws WrongSizeException {

        final double[] estimatedBiases = calibrator.getEstimatedBiases();
        assertArrayEquals(bg.getBuffer(), estimatedBiases, 0.0);

        final double[] estimatedBiases2 = new double[3];
        calibrator.getEstimatedBiases(estimatedBiases2);
        assertArrayEquals(estimatedBiases, estimatedBiases2, 0.0);

        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getEstimatedBiasesAsMatrix(bg2);

        assertEquals(bg, bg2);

        assertEquals(bg.getElementAtIndex(0), calibrator.getEstimatedBiasX(), 0.0);
        assertEquals(bg.getElementAtIndex(1), calibrator.getEstimatedBiasY(), 0.0);
        assertEquals(bg.getElementAtIndex(2), calibrator.getEstimatedBiasZ(), 0.0);

        final AngularSpeed bgx1 = calibrator.getEstimatedBiasAngularSpeedX();
        final AngularSpeed bgx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getEstimatedBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);
        assertEquals(calibrator.getEstimatedBiasX(),
                bgx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());

        final AngularSpeed bgy1 = calibrator.getEstimatedBiasAngularSpeedY();
        final AngularSpeed bgy2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getEstimatedBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);
        assertEquals(calibrator.getEstimatedBiasY(),
                bgy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());

        final AngularSpeed bgz1 = calibrator.getEstimatedBiasAngularSpeedZ();
        final AngularSpeed bgz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getEstimatedBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);
        assertEquals(calibrator.getEstimatedBiasZ(),
                bgz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());

        assertEquals(mg.getElementAt(0, 0), calibrator.getEstimatedSx(),
                0.0);
        assertEquals(mg.getElementAt(1, 1), calibrator.getEstimatedSy(),
                0.0);
        assertEquals(mg.getElementAt(2, 2), calibrator.getEstimatedSz(),
                0.0);
        assertEquals(mg.getElementAt(0, 1), calibrator.getEstimatedMxy(),
                0.0);
        assertEquals(mg.getElementAt(0, 2), calibrator.getEstimatedMxz(),
                0.0);
        assertEquals(mg.getElementAt(1, 0), calibrator.getEstimatedMyx(),
                0.0);
        assertEquals(mg.getElementAt(1, 2), calibrator.getEstimatedMyz(),
                0.0);
        assertEquals(mg.getElementAt(2, 0), calibrator.getEstimatedMzx(),
                0.0);
        assertEquals(mg.getElementAt(2, 1), calibrator.getEstimatedMzy(),
                0.0);

        assertEquals(gg, calibrator.getEstimatedGg());
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
