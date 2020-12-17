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
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.geodesic.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.geodesic.wmm.WorldMagneticModel;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDMagneticFluxDensity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.calibration.BodyMagneticFluxDensityGenerator;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.MagneticFluxDensity;
import com.irurueta.units.MagneticFluxDensityUnit;
import org.junit.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class LMedSRobustKnownHardIronAndFrameMagnetometerCalibratorTest implements
        RobustKnownHardIronAndFrameMagnetometerCalibratorListener {

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
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;

    private static final int MEASUREMENT_NUMBER = 1000;

    private static final int OUTLIER_PERCENTAGE = 20;

    private static final double THRESHOLD = 10e-9;
    private static final double LARGE_THRESHOLD = 500e-9;

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
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getHardIron(),
                new double[3], 0.0);
        final double[] hardIron = new double[3];
        calibrator.getHardIron(hardIron);
        assertArrayEquals(hardIron, new double[3], 0.0);
        assertEquals(calibrator.getHardIronMatrix(),
                new Matrix(3, 1));
        final Matrix b = new Matrix(3, 1);
        calibrator.getHardIronMatrix(b);
        assertEquals(b, new Matrix(3, 1));
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getValueY(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
                RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor2() throws WrongSizeException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
                        this);

        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getHardIron(),
                new double[3], 0.0);
        final double[] hardIron = new double[3];
        calibrator.getHardIron(hardIron);
        assertArrayEquals(hardIron, new double[3], 0.0);
        assertEquals(calibrator.getHardIronMatrix(),
                new Matrix(3, 1));
        final Matrix b = new Matrix(3, 1);
        calibrator.getHardIronMatrix(b);
        assertEquals(b, new Matrix(3, 1));
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getValueY(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
                RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor3() throws WrongSizeException {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
                        measurements);

        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getHardIron(),
                new double[3], 0.0);
        final double[] hardIron = new double[3];
        calibrator.getHardIron(hardIron);
        assertArrayEquals(hardIron, new double[3], 0.0);
        assertEquals(calibrator.getHardIronMatrix(),
                new Matrix(3, 1));
        final Matrix b = new Matrix(3, 1);
        calibrator.getHardIronMatrix(b);
        assertEquals(b, new Matrix(3, 1));
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getValueY(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
                RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor4() throws WrongSizeException {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
                        measurements, this);

        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getHardIron(),
                new double[3], 0.0);
        final double[] hardIron = new double[3];
        calibrator.getHardIron(hardIron);
        assertArrayEquals(hardIron, new double[3], 0.0);
        assertEquals(calibrator.getHardIronMatrix(),
                new Matrix(3, 1));
        final Matrix b = new Matrix(3, 1);
        calibrator.getHardIronMatrix(b);
        assertEquals(b, new Matrix(3, 1));
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getValueY(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
                RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor5() throws WrongSizeException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
                        true);

        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getHardIron(),
                new double[3], 0.0);
        final double[] hardIron = new double[3];
        calibrator.getHardIron(hardIron);
        assertArrayEquals(hardIron, new double[3], 0.0);
        assertEquals(calibrator.getHardIronMatrix(),
                new Matrix(3, 1));
        final Matrix b = new Matrix(3, 1);
        calibrator.getHardIronMatrix(b);
        assertEquals(b, new Matrix(3, 1));
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getValueY(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
                RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor6() throws WrongSizeException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
                        true, this);

        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getHardIron(),
                new double[3], 0.0);
        final double[] hardIron = new double[3];
        calibrator.getHardIron(hardIron);
        assertArrayEquals(hardIron, new double[3], 0.0);
        assertEquals(calibrator.getHardIronMatrix(),
                new Matrix(3, 1));
        final Matrix b = new Matrix(3, 1);
        calibrator.getHardIronMatrix(b);
        assertEquals(b, new Matrix(3, 1));
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getValueY(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
                RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor7() throws WrongSizeException {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
                        measurements, true);

        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getHardIron(),
                new double[3], 0.0);
        final double[] hardIron = new double[3];
        calibrator.getHardIron(hardIron);
        assertArrayEquals(hardIron, new double[3], 0.0);
        assertEquals(calibrator.getHardIronMatrix(),
                new Matrix(3, 1));
        final Matrix b = new Matrix(3, 1);
        calibrator.getHardIronMatrix(b);
        assertEquals(b, new Matrix(3, 1));
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getValueY(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
                RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testConstructor8() throws WrongSizeException {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
                        measurements, true, this);

        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownFrameMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getHardIron(),
                new double[3], 0.0);
        final double[] hardIron = new double[3];
        calibrator.getHardIron(hardIron);
        assertArrayEquals(hardIron, new double[3], 0.0);
        assertEquals(calibrator.getHardIronMatrix(),
                new Matrix(3, 1));
        final Matrix b = new Matrix(3, 1);
        calibrator.getHardIronMatrix(b);
        assertEquals(b, new Matrix(3, 1));
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getValueY(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0f);
        assertEquals(calibrator.getConfidence(),
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
                RobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS);
    }

    @Test
    public void testGetSetStopThreshold() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);

        // set new value
        calibrator.setStopThreshold(1.0);

        // check
        assertEquals(calibrator.getStopThreshold(), 1.0, 0.0);
    }

    @Test
    public void testGetSetHardIronX() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getHardIronX(), 0.0,
                0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];

        calibrator.setHardIronX(hardIronX);

        // check
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
    }

    @Test
    public void testGetSetHardIronY() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getHardIronY(), 0.0,
                0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronY = mb[1];

        calibrator.setHardIronY(hardIronY);

        // check
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
    }

    @Test
    public void testGetSetHardIronZ() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getHardIronZ(), 0.0,
                0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronZ = mb[2];

        calibrator.setHardIronZ(hardIronZ);

        // check
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
    }

    @Test
    public void testGetSetHardIronXAsMagneticFluxDensity()
            throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        final MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final MagneticFluxDensity b2 = new MagneticFluxDensity(
                hardIronX, MagneticFluxDensityUnit.TESLA);

        calibrator.setHardIronX(b2);

        // check
        final MagneticFluxDensity b3 = calibrator.getHardIronXAsMagneticFluxDensity();
        final MagneticFluxDensity b4 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b4);

        assertEquals(b2, b3);
        assertEquals(b2, b4);
    }

    @Test
    public void testGetSetHardIronYAsMagneticFluxDensity()
            throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        final MagneticFluxDensity b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronY = mb[1];
        final MagneticFluxDensity b2 = new MagneticFluxDensity(
                hardIronY, MagneticFluxDensityUnit.TESLA);

        calibrator.setHardIronY(b2);

        // check
        final MagneticFluxDensity b3 = calibrator.getHardIronYAsMagneticFluxDensity();
        final MagneticFluxDensity b4 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b4);

        assertEquals(b2, b3);
        assertEquals(b2, b4);
    }

    @Test
    public void testGetSetHardIronZAsMagneticFluxDensity()
            throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        final MagneticFluxDensity b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(MagneticFluxDensityUnit.TESLA, b1.getUnit());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronZ = mb[2];
        final MagneticFluxDensity b2 = new MagneticFluxDensity(
                hardIronZ, MagneticFluxDensityUnit.TESLA);

        calibrator.setHardIronZ(b2);

        // check
        final MagneticFluxDensity b3 = calibrator.getHardIronZAsMagneticFluxDensity();
        final MagneticFluxDensity b4 = new MagneticFluxDensity(
                1.0, MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b4);

        assertEquals(b2, b3);
        assertEquals(b2, b4);
    }

    @Test
    public void testSetHardIronCoordinates1() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default values
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        calibrator.setHardIronCoordinates(hardIronX, hardIronY, hardIronZ);

        // check
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
    }

    @Test
    public void testSetHardIronCoordinates2() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final MagneticFluxDensity hardIronX = new MagneticFluxDensity(
                hardIron[0], MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity hardIronY = new MagneticFluxDensity(
                hardIron[1], MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity hardIronZ = new MagneticFluxDensity(
                hardIron[2], MagneticFluxDensityUnit.TESLA);

        calibrator.setHardIronCoordinates(hardIronX, hardIronY, hardIronZ);

        // check
        assertEquals(calibrator.getHardIronXAsMagneticFluxDensity(), hardIronX);
        assertEquals(calibrator.getHardIronYAsMagneticFluxDensity(), hardIronY);
        assertEquals(calibrator.getHardIronZAsMagneticFluxDensity(), hardIronZ);
    }

    @Test
    public void testGetSetHardIronAsTriad() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default values
        final MagneticFluxDensityTriad triad1 = calibrator.getHardIronAsTriad();
        assertEquals(triad1.getValueX(), 0.0, 0.0);
        assertEquals(triad1.getValueY(), 0.0, 0.0);
        assertEquals(triad1.getValueZ(), 0.0, 0.0);
        assertEquals(triad1.getUnit(),
                MagneticFluxDensityUnit.TESLA);

        //set new values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final MagneticFluxDensityTriad triad2 = new MagneticFluxDensityTriad(
                MagneticFluxDensityUnit.TESLA,
                hardIronX, hardIronY, hardIronZ);
        calibrator.setHardIron(triad2);

        // check
        final MagneticFluxDensityTriad triad3 = calibrator.getHardIronAsTriad();
        final MagneticFluxDensityTriad triad4 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(triad4);

        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    public void testGetSetInitialSx() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);

        calibrator.setInitialSx(sx);

        // check
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
    }

    @Test
    public void testGetSetInitialSy() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sy = mm.getElementAt(1, 1);

        calibrator.setInitialSy(sy);

        // check
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
    }

    @Test
    public void testGetSetInitialSz() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sz = mm.getElementAt(2, 2);

        calibrator.setInitialSz(sz);

        // check
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
    }

    @Test
    public void testGetSetInitialMxy() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double mxy = mm.getElementAt(0, 1);

        calibrator.setInitialMxy(mxy);

        // check
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
    }

    @Test
    public void testGetSetInitialMxz() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double mxz = mm.getElementAt(0, 2);

        calibrator.setInitialMxz(mxz);

        // check
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
    }

    @Test
    public void testGetSetInitialMyx() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double myx = mm.getElementAt(1, 0);

        calibrator.setInitialMyx(myx);

        // check
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
    }

    @Test
    public void testGetSetInitialMyz() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double myz = mm.getElementAt(1, 2);

        calibrator.setInitialMyz(myz);

        // check
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
    }

    @Test
    public void testGetSetInitialMzx() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double mzx = mm.getElementAt(2, 0);

        calibrator.setInitialMzx(mzx);

        // check
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
    }

    @Test
    public void testGetSetInitialMzy() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double mzy = mm.getElementAt(2, 1);

        calibrator.setInitialMzy(mzy);

        // check
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
    }

    @Test
    public void testSetInitialScalingFactors() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default values
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);

        // set new values
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);

        calibrator.setInitialScalingFactors(sx, sy, sz);

        // check
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
    }

    @Test
    public void testSetInitialCrossCouplingErrors() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default values
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        calibrator.setInitialCrossCouplingErrors(mxy, mxz, myx,
                myz, mzx, mzy);

        // check
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
    }

    @Test
    public void testSetInitialScalingFactorsAndCrossCouplingErrors()
            throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

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

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);
        final double mxy = mm.getElementAt(0, 1);
        final double mxz = mm.getElementAt(0, 2);
        final double myx = mm.getElementAt(1, 0);
        final double myz = mm.getElementAt(1, 2);
        final double mzx = mm.getElementAt(2, 0);
        final double mzy = mm.getElementAt(2, 1);

        calibrator.setInitialScalingFactorsAndCrossCouplingErrors(
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
    }

    @Test
    public void testGetHardIronAsArray() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertArrayEquals(calibrator.getHardIron(),
                new double[3], 0.0);
        final double[] result1 = new double[3];
        calibrator.getHardIron(result1);
        assertArrayEquals(result1, new double[3], 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] bm = generateHardIron(randomizer);
        calibrator.setHardIron(bm);

        // check
        assertArrayEquals(calibrator.getHardIron(), bm, 0.0);
        final double[] result2 = new double[3];
        calibrator.getHardIron(result2);
        assertArrayEquals(result2, bm, 0.0);

        // Force IllegalArgumentException
        try {
            calibrator.getHardIron(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setHardIron(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetHardIronAsMatrix() throws LockedException,
            WrongSizeException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getHardIronMatrix(),
                new Matrix(3, 1));
        final Matrix result1 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(result1);
        assertEquals(result1, new Matrix(3, 1));

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] bm = generateHardIron(randomizer);
        final Matrix b = Matrix.newFromArray(bm);
        calibrator.setHardIron(b);

        // check
        assertEquals(calibrator.getHardIronMatrix(), b);
        final Matrix result2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(result2);
        assertEquals(result2, b);

        // Force IllegalArgumentException
        try {
            calibrator.getHardIronMatrix(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.getHardIronMatrix(new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setHardIron(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setHardIron(new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetInitialMm() throws WrongSizeException,
            LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMm(), new Matrix(3, 3));
        final Matrix result1 = new Matrix(3, 3);
        calibrator.getInitialMm(result1);

        // set new value
        final Matrix mm = generateSoftIronGeneral();
        calibrator.setInitialMm(mm);

        // check
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix result2 = new Matrix(3, 3);
        calibrator.getInitialMm(result2);
        assertEquals(mm, result2);
    }

    @Test
    public void testGetSetMeasurements() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getMeasurements());

        // set new value
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        calibrator.setMeasurements(measurements);

        // check
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testIsSetCommonAxisUsed() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testGetListener() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testIsReady() throws LockedException, IOException,
            InvalidSourceAndDestinationFrameTypeException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check initial value
        assertFalse(calibrator.isReady());

        // set not enough measurements
        calibrator.setMeasurements(Collections
                .<StandardDeviationFrameBodyMagneticFluxDensity>emptyList());

        // check
        assertFalse(calibrator.isReady());

        // set enough measurements
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final Matrix softIron = generateSoftIronGeneral();
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                generateMeasurementsMultipleOrientationswithSamePosition(
                        hardIron, softIron,
                        wmmEstimator, randomizer);

        calibrator.setMeasurements(measurements);

        // check
        assertTrue(calibrator.isReady());
    }

    @Test
    public void testGetSetMagneticModel() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getMagneticModel());

        // set new value
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        calibrator.setMagneticModel(magneticModel);

        // check
        assertSame(calibrator.getMagneticModel(), magneticModel);
    }

    @Test
    public void testIsSetLinearCalibratorUsed() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertTrue(calibrator.isLinearCalibratorUsed());

        // set new value
        calibrator.setLinearCalibratorUsed(false);

        // check
        assertFalse(calibrator.isLinearCalibratorUsed());
    }

    @Test
    public void testIsSetPreliminarySolutionRefined() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertFalse(calibrator.isPreliminarySolutionRefined());

        // set new value
        calibrator.setPreliminarySolutionRefined(true);

        // check
        assertTrue(calibrator.isPreliminarySolutionRefined());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

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
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

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
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

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
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertTrue(calibrator.isResultRefined());

        // set new value
        calibrator.setResultRefined(false);

        // check
        assertFalse(calibrator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertTrue(calibrator.isCovarianceKept());

        // set new value
        calibrator.setCovarianceKept(false);

        // check
        assertFalse(calibrator.isCovarianceKept());
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertNull(calibrator.getQualityScores());

        // set new value
        calibrator.setQualityScores(new double[3]);

        // check
        assertNull(calibrator.getQualityScores());
    }

    @Test
    public void testGetSetPreliminarySubsetSize() throws LockedException {
        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator();

        // check default value
        assertEquals(calibrator.getPreliminarySubsetSize(),
                LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS);

        // set new value
        calibrator.setPreliminarySubsetSize(4);

        // check
        assertEquals(calibrator.getPreliminarySubsetSize(), 4);

        // Force IllegalArgumentException
        try {
            calibrator.setPreliminarySubsetSize(2);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testCalibrateGeneralNoNoiseInlier()
            throws IOException,
            InvalidSourceAndDestinationFrameTypeException,
            LockedException, CalibrationException, NotReadyException,
            WrongSizeException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, MAGNETOMETER_NOISE_STD);


        final NEDPosition position = createPosition(randomizer);
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                new ArrayList<>();
        for (int i = 0; i < MEASUREMENT_NUMBER; i++) {

            final StandardDeviationFrameBodyMagneticFluxDensity b;
            if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                // outlier
                b = generateMeasureAtPosition(hardIron, mm, wmmEstimator,
                        randomizer, noiseRandomizer, position);
            } else {
                // inlier
                b = generateMeasureAtPosition(hardIron, mm, wmmEstimator,
                        randomizer, null, position);
            }
            measurements.add(b);
        }

        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
                        measurements, false, this);
        calibrator.setHardIron(hardIron);
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

        final Matrix estimatedMm = calibrator.getEstimatedMm();

        assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMm, calibrator);

        assertNotNull(calibrator.getEstimatedCovariance());
    }

    @Test
    public void testCalibrateCommonAxisNoNoiseInlier()
            throws IOException,
            InvalidSourceAndDestinationFrameTypeException,
            LockedException, CalibrationException, NotReadyException,
            WrongSizeException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix mm = generateSoftIronCommonAxis();
        assertNotNull(mm);

        final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                new Random(), 0.0, MAGNETOMETER_NOISE_STD);


        final NEDPosition position = createPosition(randomizer);
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                new ArrayList<>();
        for (int i = 0; i < MEASUREMENT_NUMBER; i++) {

            final StandardDeviationFrameBodyMagneticFluxDensity b;
            if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                // outlier
                b = generateMeasureAtPosition(hardIron, mm, wmmEstimator,
                        randomizer, noiseRandomizer, position);
            } else {
                // inlier
                b = generateMeasureAtPosition(hardIron, mm, wmmEstimator,
                        randomizer, null, position);
            }
            measurements.add(b);
        }

        final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
                        measurements, true, this);
        calibrator.setHardIron(hardIron);
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

        final Matrix estimatedMm = calibrator.getEstimatedMm();

        assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMm, calibrator);

        assertNotNull(calibrator.getEstimatedCovariance());
    }

    @Test
    public void testCalibrateGeneralWithInlierNoise()
            throws IOException,
            InvalidSourceAndDestinationFrameTypeException,
            LockedException, CalibrationException, NotReadyException,
            WrongSizeException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                    new WMMEarthMagneticFluxDensityEstimator();

            final double[] hardIron = generateHardIron(randomizer);
            final Matrix mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final GaussianRandomizer inlierNoiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, MAGNETOMETER_NOISE_STD);
            final GaussianRandomizer outlierNoiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0,
                    OUTLIER_ERROR_FACTOR * MAGNETOMETER_NOISE_STD);


            final NEDPosition position = createPosition(randomizer);
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                    new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {

                final StandardDeviationFrameBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasureAtPosition(hardIron, mm, wmmEstimator,
                            randomizer, outlierNoiseRandomizer, position);
                } else {
                    // inlier
                    b = generateMeasureAtPosition(hardIron, mm, wmmEstimator,
                            randomizer, inlierNoiseRandomizer, position);
                }
                measurements.add(b);
            }

            final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                    new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
                            measurements, false, this);
            calibrator.setHardIron(hardIron);
            calibrator.setStopThreshold(LARGE_THRESHOLD);

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

            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateCommonAxisWithInlierNoise()
            throws IOException,
            InvalidSourceAndDestinationFrameTypeException,
            LockedException, CalibrationException, NotReadyException,
            WrongSizeException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                    new WMMEarthMagneticFluxDensityEstimator();

            final double[] hardIron = generateHardIron(randomizer);
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final GaussianRandomizer inlierNoiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, MAGNETOMETER_NOISE_STD);
            final GaussianRandomizer outlierNoiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0,
                    OUTLIER_ERROR_FACTOR * MAGNETOMETER_NOISE_STD);


            final NEDPosition position = createPosition(randomizer);
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                    new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {

                final StandardDeviationFrameBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasureAtPosition(hardIron, mm, wmmEstimator,
                            randomizer, outlierNoiseRandomizer, position);
                } else {
                    // inlier
                    b = generateMeasureAtPosition(hardIron, mm, wmmEstimator,
                            randomizer, inlierNoiseRandomizer, position);
                }
                measurements.add(b);
            }

            final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                    new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
                            measurements, true, this);
            calibrator.setHardIron(hardIron);
            calibrator.setStopThreshold(LARGE_THRESHOLD);

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

            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralNoRefinement()
            throws IOException,
            InvalidSourceAndDestinationFrameTypeException,
            LockedException, CalibrationException, NotReadyException,
            WrongSizeException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                    new WMMEarthMagneticFluxDensityEstimator();

            final double[] hardIron = generateHardIron(randomizer);
            final Matrix mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, MAGNETOMETER_NOISE_STD);


            final NEDPosition position = createPosition(randomizer);
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                    new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {

                final StandardDeviationFrameBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasureAtPosition(hardIron, mm, wmmEstimator,
                            randomizer, noiseRandomizer, position);
                } else {
                    // inlier
                    b = generateMeasureAtPosition(hardIron, mm, wmmEstimator,
                            randomizer, null, position);
                }
                measurements.add(b);
            }

            final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                    new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
                            measurements, false, this);
            calibrator.setHardIron(hardIron);
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

            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

            assertNull(calibrator.getEstimatedCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralNonLinearWithInitialValue()
            throws IOException,
            InvalidSourceAndDestinationFrameTypeException,
            LockedException, CalibrationException, NotReadyException,
            WrongSizeException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                    new WMMEarthMagneticFluxDensityEstimator();

            final double[] hardIron = generateHardIron(randomizer);
            final Matrix mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, MAGNETOMETER_NOISE_STD);


            final NEDPosition position = createPosition(randomizer);
            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                    new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {

                final StandardDeviationFrameBodyMagneticFluxDensity b;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    b = generateMeasureAtPosition(hardIron, mm, wmmEstimator,
                            randomizer, noiseRandomizer, position);
                } else {
                    // inlier
                    b = generateMeasureAtPosition(hardIron, mm, wmmEstimator,
                            randomizer, null, position);
                }
                measurements.add(b);
            }

            final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator =
                    new LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator(
                            measurements, false, this);
            calibrator.setHardIron(hardIron);
            calibrator.setStopThreshold(THRESHOLD);
            calibrator.setInitialMm(mm);
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

            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(
            final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator) {
        checkLocked((LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator) calibrator);
        mCalibrateStart++;
    }

    @Override
    public void onCalibrateEnd(
            final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator) {
        checkLocked((LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator) calibrator);
        mCalibrateEnd++;
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator,
            final int iteration) {
        checkLocked((LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator) calibrator);
        mCalibrateNextIteration++;
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownHardIronAndFrameMagnetometerCalibrator calibrator,
            final float progress) {
        checkLocked((LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator) calibrator);
        mCalibrateProgressChange++;
    }

    private void reset() {
        mCalibrateStart = 0;
        mCalibrateEnd = 0;
        mCalibrateNextIteration = 0;
        mCalibrateProgressChange = 0;
    }

    private void checkLocked(
            final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
        try {
            calibrator.setStopThreshold(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setHardIronX(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setHardIronY(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setHardIronZ(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setHardIronX(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setHardIronY(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setHardIronZ(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setHardIronCoordinates(
                    0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setHardIronCoordinates(
                    null, null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setHardIron((MagneticFluxDensityTriad) null);
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
            calibrator.setHardIron((double[]) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setHardIron((Matrix) null);
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
            final Matrix mm,
            final LMedSRobustKnownHardIronAndFrameMagnetometerCalibrator calibrator) {

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

    private static List<StandardDeviationFrameBodyMagneticFluxDensity>
    generateMeasurementsMultipleOrientationswithSamePosition(
            final double[] hardIron, final Matrix softIron,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final UniformRandomizer randomizer)
            throws InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition position = createPosition(randomizer);
        final List<StandardDeviationFrameBodyMagneticFluxDensity> result =
                new ArrayList<>();
        for (int i = 0; i < RobustKnownFrameMagnetometerCalibrator.MINIMUM_MEASUREMENTS; i++) {
            result.add(generateMeasureAtPosition(hardIron, softIron,
                    wmmEstimator, randomizer, null, position));
        }
        return result;
    }

    private static StandardDeviationFrameBodyMagneticFluxDensity generateMeasureAtPosition(
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

    private static StandardDeviationFrameBodyMagneticFluxDensity generateMeasure(
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

        final double std = noiseRandomizer != null ?
                noiseRandomizer.getStandardDeviation() :
                MAGNETOMETER_NOISE_STD;
        return new StandardDeviationFrameBodyMagneticFluxDensity(
                measuredMagnetic, frame, timestamp, std);
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
