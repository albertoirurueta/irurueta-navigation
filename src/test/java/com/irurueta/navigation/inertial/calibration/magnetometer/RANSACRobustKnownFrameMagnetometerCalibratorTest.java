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
package com.irurueta.navigation.inertial.calibration.magnetometer;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.geodesic.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.calibration.BodyMagneticFluxDensityGenerator;
import com.irurueta.navigation.inertial.calibration.FrameBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collections;
import java.util.Date;
import java.util.List;

import static org.junit.Assert.*;

public class RANSACRobustKnownFrameMagnetometerCalibratorTest implements
        RobustKnownFrameMagnetometerCalibratorListener {

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

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-5;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;

    private static final int MEASUREMENT_NUMBER = 1000;
    private static final int LARGE_MEASUREMENT_NUMBER = 100000;

    private static final int OUTLIER_PERCENTAGE = 20;

    private static final double THRESHOLD = 1e-4;
    private static final double LARGE_THRESHOLD = 5e-2;

    private static final double OUTLIER_ERROR_FACTOR = 100.0;

    private static final int TIMES = 100;

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

    private int mCalibrateStart;
    private int mCalibrateEnd;
    private int mCalibrateNextIteration;
    private int mCalibrateProgressChange;

    @Test
    public void testConstructor1() throws WrongSizeException {
        final RANSACRobustKnownFrameMagnetometerCalibrator calibrator =
                new RANSACRobustKnownFrameMagnetometerCalibrator();

        assertEquals(calibrator.getThreshold(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(),
                new double[3], 0.0);
        final double[] hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(hardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix b = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(b);
        assertEquals(b, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(mm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
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
                RobustKnownFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor2() throws WrongSizeException {
        final RANSACRobustKnownFrameMagnetometerCalibrator calibrator =
                new RANSACRobustKnownFrameMagnetometerCalibrator(
                        this);

        assertEquals(calibrator.getThreshold(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(),
                new double[3], 0.0);
        final double[] hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(hardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix b = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(b);
        assertEquals(b, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(mm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
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
                RobustKnownFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor3() throws WrongSizeException {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RANSACRobustKnownFrameMagnetometerCalibrator calibrator =
                new RANSACRobustKnownFrameMagnetometerCalibrator(
                        measurements);

        assertEquals(calibrator.getThreshold(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(),
                new double[3], 0.0);
        final double[] hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(hardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix b = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(b);
        assertEquals(b, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(mm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
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
                RobustKnownFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor4() throws WrongSizeException {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RANSACRobustKnownFrameMagnetometerCalibrator calibrator =
                new RANSACRobustKnownFrameMagnetometerCalibrator(
                        measurements, this);

        assertEquals(calibrator.getThreshold(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(),
                new double[3], 0.0);
        final double[] hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(hardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix b = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(b);
        assertEquals(b, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(mm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
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
                RobustKnownFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor5() throws WrongSizeException {
        final RANSACRobustKnownFrameMagnetometerCalibrator calibrator =
                new RANSACRobustKnownFrameMagnetometerCalibrator(
                        true);

        assertEquals(calibrator.getThreshold(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(),
                new double[3], 0.0);
        final double[] hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(hardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix b = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(b);
        assertEquals(b, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(mm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
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
                RobustKnownFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor6() throws WrongSizeException {
        final RANSACRobustKnownFrameMagnetometerCalibrator calibrator =
                new RANSACRobustKnownFrameMagnetometerCalibrator(
                        true, this);

        assertEquals(calibrator.getThreshold(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(),
                new double[3], 0.0);
        final double[] hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(hardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix b = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(b);
        assertEquals(b, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(mm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
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
                RobustKnownFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor7() throws WrongSizeException {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RANSACRobustKnownFrameMagnetometerCalibrator calibrator =
                new RANSACRobustKnownFrameMagnetometerCalibrator(
                        measurements, true);

        assertEquals(calibrator.getThreshold(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(),
                new double[3], 0.0);
        final double[] hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(hardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix b = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(b);
        assertEquals(b, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(mm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
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
                RobustKnownFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor8() throws WrongSizeException {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final RANSACRobustKnownFrameMagnetometerCalibrator calibrator =
                new RANSACRobustKnownFrameMagnetometerCalibrator(
                        measurements, true, this);

        assertEquals(calibrator.getThreshold(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_THRESHOLD,
                0.0);
        assertEquals(calibrator.isComputeAndKeepInliersEnabled(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_INLIERS);
        assertEquals(calibrator.isComputeAndKeepResiduals(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_COMPUTE_AND_KEEP_RESIDUALS);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.RANSAC);
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(),
                new double[3], 0.0);
        final double[] hardIron = new double[3];
        calibrator.getInitialHardIron(hardIron);
        assertArrayEquals(hardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix b = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(b);
        assertEquals(b, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix mm = new Matrix(3, 3);
        calibrator.getInitialMm(mm);
        assertEquals(mm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertTrue(calibrator.isLinearCalibratorUsed());
        assertFalse(calibrator.isPreliminarySolutionRefined());
        assertEquals(calibrator.getProgressDelta(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertNull(calibrator.getEstimatedHardIron());
        assertFalse(calibrator.getEstimatedHardIron(null));
        assertNull(calibrator.getEstimatedHardIronAsMatrix());
        assertFalse(calibrator.getEstimatedHardIronAsMatrix(null));
        assertNull(calibrator.getEstimatedHardIronX());
        assertNull(calibrator.getEstimatedHardIronY());
        assertNull(calibrator.getEstimatedHardIronZ());
        assertNull(calibrator.getEstimatedMm());
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
                RobustKnownFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testGetSetThreshold() throws LockedException {
        final RANSACRobustKnownFrameMagnetometerCalibrator calibrator =
                new RANSACRobustKnownFrameMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getThreshold(),
                RANSACRobustKnownFrameMagnetometerCalibrator.DEFAULT_THRESHOLD,
                0.0);

        // set new value
        calibrator.setThreshold(1.0);

        // check
        assertEquals(calibrator.getThreshold(), 1.0, 0.0);
    }

    @Test
    public void testIsSetComputeAndKeepInliersEnabled() {

    }

    @Override
    public void onCalibrateStart(
            final RobustKnownFrameMagnetometerCalibrator calibrator) {
        checkLocked((RANSACRobustKnownFrameMagnetometerCalibrator) calibrator);
        mCalibrateStart++;
    }

    @Override
    public void onCalibrateEnd(
            final RobustKnownFrameMagnetometerCalibrator calibrator) {
        checkLocked((RANSACRobustKnownFrameMagnetometerCalibrator) calibrator);
        mCalibrateEnd++;
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownFrameMagnetometerCalibrator calibrator,
            final int iteration) {
        checkLocked((RANSACRobustKnownFrameMagnetometerCalibrator) calibrator);
        mCalibrateNextIteration++;
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownFrameMagnetometerCalibrator calibrator,
            final float progress) {
        checkLocked((RANSACRobustKnownFrameMagnetometerCalibrator) calibrator);
        mCalibrateProgressChange++;
    }

    private void reset() {
        mCalibrateStart = 0;
        mCalibrateEnd = 0;
    }

    private void checkLocked(
            final RANSACRobustKnownFrameMagnetometerCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
        try {
            calibrator.setThreshold(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setComputeAndKeepInliersEnabled(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setComputeAndKeepResidualsEnabled(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialHardIronX(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialHardIronY(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialHardIronZ(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialHardIron(
                    0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialSx(0.0);
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
            calibrator.setInitialScalingFactors(
                    0.0, 0.0, 0.0);
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
            calibrator.setInitialHardIron((double[]) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialHardIron((Matrix) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialMm(null);
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
            calibrator.setMagneticModel(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setLinearCalibratorUsed(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setPreliminarySolutionRefined(false);
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
            calibrator.setMaxIterations(100);
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
            calibrator.calibrate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
    }

    private void assertEstimatedResult(
            final Matrix hardIron, final Matrix mm,
            final KnownFrameMagnetometerLinearLeastSquaresCalibrator calibrator)
            throws WrongSizeException {

        final double[] estimatedHardIron = calibrator.getEstimatedHardIron();
        assertArrayEquals(hardIron.getBuffer(), estimatedHardIron, 0.0);

        final double[] estimatedHardIron2 = new double[3];
        assertTrue(calibrator.getEstimatedHardIron(estimatedHardIron2));
        assertArrayEquals(estimatedHardIron, estimatedHardIron2, 0.0);

        final Matrix hardIron2 = new Matrix(3, 1);
        assertTrue(calibrator.getEstimatedHardIronAsMatrix(hardIron2));

        assertEquals(hardIron, hardIron2);

        assertEquals(hardIron.getElementAtIndex(0),
                calibrator.getEstimatedHardIronX(), 0.0);
        assertEquals(hardIron.getElementAtIndex(1),
                calibrator.getEstimatedHardIronY(), 0.0);
        assertEquals(hardIron.getElementAtIndex(2),
                calibrator.getEstimatedHardIronZ(), 0.0);

        assertEquals(mm.getElementAt(0, 0), calibrator.getEstimatedSx(),
                0.0);
        assertEquals(mm.getElementAt(1, 1), calibrator.getEstimatedSy(),
                0.0);
        assertEquals(mm.getElementAt(2, 2), calibrator.getEstimatedSz(),
                0.0);
        assertEquals(mm.getElementAt(0, 1), calibrator.getEstimatedMxy(),
                0.0);
        assertEquals(mm.getElementAt(0, 2), calibrator.getEstimatedMxz(),
                0.0);
        assertEquals(mm.getElementAt(1, 0), calibrator.getEstimatedMyx(),
                0.0);
        assertEquals(mm.getElementAt(1, 2), calibrator.getEstimatedMyz(),
                0.0);
        assertEquals(mm.getElementAt(2, 0), calibrator.getEstimatedMzx(),
                0.0);
        assertEquals(mm.getElementAt(2, 1), calibrator.getEstimatedMzy(),
                0.0);
    }

    private static List<FrameBodyMagneticFluxDensity>
    generateMeasurementsMultipleOrientationswithSamePosition(
            final double[] hardIron, final Matrix softIron,
            final int numberOfMeasurements,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final UniformRandomizer randomizer,
            final GaussianRandomizer noiseRandomizer)
            throws InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition position = createPosition(randomizer);
        final List<FrameBodyMagneticFluxDensity> result =
                new ArrayList<>();
        for (int i = 0; i < numberOfMeasurements; i++) {
            result.add(generateMeasureAtPosition(hardIron, softIron,
                    wmmEstimator, randomizer, noiseRandomizer, position));
        }
        return result;
    }

    private static List<FrameBodyMagneticFluxDensity>
    generateMeasurementsMultiplePositionsWithSameOrientation(
            final double[] hardIron, final Matrix softIron,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final UniformRandomizer randomizer)
            throws InvalidSourceAndDestinationFrameTypeException {
        final CoordinateTransformation cnb = new CoordinateTransformation(
                FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final List<FrameBodyMagneticFluxDensity> result =
                new ArrayList<>();
        for (int i = 0; i < KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {
            result.add(generateMeasureAtOrientation(hardIron, softIron,
                    wmmEstimator, randomizer, cnb));
        }
        return result;
    }

    private static FrameBodyMagneticFluxDensity generateMeasureAtOrientation(
            final double[] hardIron, final Matrix softIron,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final UniformRandomizer randomizer,
            final CoordinateTransformation cnb)
            throws InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition position = createPosition(randomizer);
        return generateMeasure(hardIron, softIron, wmmEstimator,
                randomizer, null, position, cnb);
    }

    private static FrameBodyMagneticFluxDensity generateMeasureAtPosition(
            final double[] hardIron, final Matrix softIron,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final UniformRandomizer randomizer,
            final GaussianRandomizer noiseRandomizer,
            final NEDPosition position)
            throws InvalidSourceAndDestinationFrameTypeException {
        final CoordinateTransformation cnb = generateBodyC(randomizer);
        return generateMeasure(hardIron, softIron, wmmEstimator,
                randomizer, noiseRandomizer, position, cnb);
    }

    private static FrameBodyMagneticFluxDensity generateMeasure(
            final double[] hardIron, final Matrix softIron,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final UniformRandomizer randomizer,
            final GaussianRandomizer noiseRandomizer,
            final NEDPosition position,
            final CoordinateTransformation cnb)
            throws InvalidSourceAndDestinationFrameTypeException {

        final Date timestamp = new Date(createTimestamp(randomizer));
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

        final CoordinateTransformation cbn = cnb.inverseAndReturnNew();
        final NEDFrame frame = new NEDFrame(position, cbn);

        return new FrameBodyMagneticFluxDensity(measuredMagnetic, frame,
                timestamp);
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
