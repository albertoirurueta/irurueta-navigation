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
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collection;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class KnownFrameMagnetometerNonLinearLeastSquaresCalibratorTest implements
        KnownFrameMagnetometerNonLinearLeastSquaresCalibratorListener {

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

    private static final int SMALL_MEASUREMENT_NUMBER = 16;
    private static final int LARGE_MEASUREMENT_NUMBER = 100000;

    private static final double MAGNETOMETER_NOISE_STD = 1e-8;

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-5;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;

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

    @Test
    public void testConstructor1() throws WrongSizeException {
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), new double[3],
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor2() throws WrongSizeException {
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), new double[3],
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor3() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements = Collections.emptyList();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(measurements);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), new double[3],
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor4() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(measurements,
                        this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), new double[3],
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor5() throws WrongSizeException {
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), new double[3],
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor6() throws WrongSizeException {
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), new double[3],
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor7() throws WrongSizeException {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), new double[3],
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor8() throws WrongSizeException {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), new double[3],
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor9() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        magneticModel);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), new double[3],
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor10() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        magneticModel, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), new double[3],
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor11() throws WrongSizeException {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, magneticModel);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), new double[3],
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor12() throws WrongSizeException {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, magneticModel, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), new double[3],
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor13() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, magneticModel);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), new double[3],
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor14() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, magneticModel, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), new double[3],
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor15() throws WrongSizeException {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), new double[3],
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor16() throws WrongSizeException {
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel,
                        this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), new double[3],
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, new double[3], 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, new Matrix(3, 1));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor17() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        hardIronX, hardIronY, hardIronZ);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor18() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        hardIronX, hardIronY, hardIronZ, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor19() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, hardIronX, hardIronY, hardIronZ);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor20() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, hardIronX, hardIronY, hardIronZ, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor21() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, hardIronX, hardIronY, hardIronZ);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor22() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, hardIronX, hardIronY,
                        hardIronZ, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor23() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, hardIronX,
                        hardIronY, hardIronZ);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor24() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, hardIronX,
                        hardIronY, hardIronZ, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor25() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        magneticModel, hardIronX, hardIronY, hardIronZ);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor26() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        magneticModel, hardIronX, hardIronY, hardIronZ,
                        this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor27() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, magneticModel, hardIronX, hardIronY,
                        hardIronZ);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor28() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, magneticModel, hardIronX, hardIronY,
                        hardIronZ, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor29() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, magneticModel, hardIronX,
                        hardIronY, hardIronZ);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor30() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, magneticModel, hardIronX,
                        hardIronY, hardIronZ, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor31() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel,
                        hardIronX, hardIronY, hardIronZ);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor32() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel,
                        hardIronX, hardIronY, hardIronZ, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor33() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);

        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        hardIronX, hardIronY, hardIronZ, sx, sy, sz);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                Matrix.diagonal(new double[]{ sx, sy, sz }));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, Matrix.diagonal(new double[]{ sx, sy, sz }));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor34() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);

        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        hardIronX, hardIronY, hardIronZ, sx, sy, sz,
                        this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                Matrix.diagonal(new double[]{ sx, sy, sz }));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, Matrix.diagonal(new double[]{ sx, sy, sz }));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor35() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, hardIronX, hardIronY, hardIronZ,
                        sx, sy, sz);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                Matrix.diagonal(new double[]{ sx, sy, sz }));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, Matrix.diagonal(new double[]{ sx, sy, sz }));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor36() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, hardIronX, hardIronY, hardIronZ,
                        sx, sy, sz, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                Matrix.diagonal(new double[]{ sx, sy, sz }));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, Matrix.diagonal(new double[]{ sx, sy, sz }));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor37() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);

        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, hardIronX, hardIronY,
                        hardIronZ, sx, sy, sz);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                Matrix.diagonal(new double[]{ sx, sy, sz }));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, Matrix.diagonal(new double[]{ sx, sy, sz }));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor38() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);

        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, hardIronX, hardIronY,
                        hardIronZ, sx, sy, sz, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                Matrix.diagonal(new double[]{ sx, sy, sz }));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, Matrix.diagonal(new double[]{ sx, sy, sz }));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor39() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, hardIronX,
                        hardIronY, hardIronZ, sx, sy, sz);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                Matrix.diagonal(new double[]{ sx, sy, sz }));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, Matrix.diagonal(new double[]{ sx, sy, sz }));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor40() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, hardIronX,
                        hardIronY, hardIronZ, sx, sy, sz, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                Matrix.diagonal(new double[]{ sx, sy, sz }));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, Matrix.diagonal(new double[]{ sx, sy, sz }));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor41() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        magneticModel, hardIronX, hardIronY, hardIronZ,
                        sx, sy, sz);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                Matrix.diagonal(new double[]{ sx, sy, sz }));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, Matrix.diagonal(new double[]{ sx, sy, sz }));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor42() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        magneticModel, hardIronX, hardIronY, hardIronZ,
                        sx, sy, sz, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                Matrix.diagonal(new double[]{ sx, sy, sz }));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, Matrix.diagonal(new double[]{ sx, sy, sz }));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor43() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, magneticModel, hardIronX, hardIronY,
                        hardIronZ, sx, sy, sz);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                Matrix.diagonal(new double[]{ sx, sy, sz }));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, Matrix.diagonal(new double[]{ sx, sy, sz }));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor44() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, magneticModel, hardIronX, hardIronY,
                        hardIronZ, sx, sy, sz, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                Matrix.diagonal(new double[]{ sx, sy, sz }));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, Matrix.diagonal(new double[]{ sx, sy, sz }));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor45() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, magneticModel, hardIronX, hardIronY,
                        hardIronZ, sx, sy, sz);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                Matrix.diagonal(new double[]{ sx, sy, sz }));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, Matrix.diagonal(new double[]{ sx, sy, sz }));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor46() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, magneticModel, hardIronX, hardIronY,
                        hardIronZ, sx, sy, sz, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                Matrix.diagonal(new double[]{ sx, sy, sz }));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, Matrix.diagonal(new double[]{ sx, sy, sz }));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor47() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel,
                        hardIronX, hardIronY, hardIronZ, sx, sy, sz);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                Matrix.diagonal(new double[]{ sx, sy, sz }));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, Matrix.diagonal(new double[]{ sx, sy, sz }));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor48() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final double sx = mm.getElementAt(0, 0);
        final double sy = mm.getElementAt(1, 1);
        final double sz = mm.getElementAt(2, 2);

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel,
                        hardIronX, hardIronY, hardIronZ, sx, sy, sz,
                        this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                Matrix.diagonal(new double[]{ sx, sy, sz }));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, Matrix.diagonal(new double[]{ sx, sy, sz }));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor49() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
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

        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        hardIronX, hardIronY, hardIronZ, sx, sy, sz,
                        mxy, mxz, myx, myz, mzx, mzy);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor50() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
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

        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        hardIronX, hardIronY, hardIronZ, sx, sy, sz,
                        mxy, mxz, myx, myz, mzx, mzy,this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor51() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
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

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, hardIronX, hardIronY, hardIronZ,
                        sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor52() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
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

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, hardIronX, hardIronY, hardIronZ,
                        sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
                        this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor53() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
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

        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, hardIronX, hardIronY, hardIronZ, sx, sy, sz,
                        mxy, mxz, myx, myz, mzx, mzy);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor54() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
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

        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, hardIronX, hardIronY, hardIronZ, sx, sy, sz,
                        mxy, mxz, myx, myz, mzx, mzy, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor55() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
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

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, hardIronX,
                        hardIronY, hardIronZ, sx, sy, sz, mxy, mxz, myx,
                        myz, mzx, mzy);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor56() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
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

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, hardIronX,
                        hardIronY, hardIronZ, sx, sy, sz, mxy, mxz, myx,
                        myz, mzx, mzy, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor57() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
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

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        magneticModel, hardIronX, hardIronY, hardIronZ,
                        sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor58() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
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

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        magneticModel, hardIronX, hardIronY, hardIronZ,
                        sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
                        this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor59() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
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

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, magneticModel, hardIronX, hardIronY,
                        hardIronZ, sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor60() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
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

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, magneticModel, hardIronX, hardIronY,
                        hardIronZ, sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy,
                        this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor61() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
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

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, magneticModel, hardIronX,
                        hardIronY, hardIronZ, sx, sy, sz, mxy, mxz, myx,
                        myz, mzx, mzy);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor62() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
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

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, magneticModel, hardIronX,
                        hardIronY, hardIronZ, sx, sy, sz, mxy, mxz, myx,
                        myz, mzx, mzy, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor63() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
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

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel,
                        hardIronX, hardIronY, hardIronZ, sx, sy, sz,
                        mxy, mxz, myx, myz, mzx, mzy);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor64() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
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

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel,
                        hardIronX, hardIronY, hardIronZ, sx, sy, sz,
                        mxy, mxz, myx, myz, mzx, mzy, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor65() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        mb);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force Illegal ArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor66() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        mb, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor67() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, mb);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor68() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, mb, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor69() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, mb);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor70() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, mb, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor71() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, mb);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor72() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, mb,
                        this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, new double[1],
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor73() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        magneticModel, mb);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    magneticModel, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor74() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        magneticModel, mb, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    magneticModel, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor75() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, magneticModel, mb);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, magneticModel, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor76() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, magneticModel, mb, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, magneticModel, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor77() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, magneticModel, mb);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, magneticModel, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor78() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, magneticModel, mb,
                        this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, magneticModel, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor79() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel,
                        mb);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, magneticModel, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor80() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel,
                        mb, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, magneticModel,
                    new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor81() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);

        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        mbm);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force Illegal ArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor82() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);

        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        mbm, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    new Matrix(1, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    new Matrix(3, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor83() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, mbm);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor84() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, mbm, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(1, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(3, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor85() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);

        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, mbm);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor86() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);

        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, mbm, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, new Matrix(1, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, new Matrix(3, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor87() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, mbm);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor88() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, mbm,
                        this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, new Matrix(1, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, new Matrix(3, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor89() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        magneticModel, mbm);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    magneticModel, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    magneticModel, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor90() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        magneticModel, mbm, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    magneticModel, new Matrix(1, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    magneticModel, new Matrix(3, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor91() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, magneticModel, mbm);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, magneticModel, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, magneticModel, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor92() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, magneticModel, mbm, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, magneticModel,
                    new Matrix(1, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, magneticModel,
                    new Matrix(3, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor93() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, magneticModel, mbm);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, magneticModel, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, magneticModel, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor94() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, magneticModel, mbm,
                        this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, magneticModel,
                    new Matrix(1, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, magneticModel,
                    new Matrix(3, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor95() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel,
                        mbm);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, magneticModel,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, magneticModel,
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor96() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel,
                        mbm, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(),
                new Matrix(3, 3));
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, new Matrix(3, 3));
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, magneticModel,
                    new Matrix(1, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, magneticModel,
                    new Matrix(3, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor97() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);
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

        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        mbm, mm);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force Illegal ArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    new Matrix(1, 1), mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    new Matrix(3, 3), mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    mbm, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    mbm, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor98() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);
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

        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        mbm, mm, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    new Matrix(1, 1), mm, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    new Matrix(3, 3), mm, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    mbm, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    mbm, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor99() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);
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

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, mbm, mm);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(1, 1), mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(3, 3), mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, mbm, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, mbm, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor100() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);
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

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, mbm, mm, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(1, 1), mm,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(3, 3), mm,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, mbm, new Matrix(1, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, mbm, new Matrix(3, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor101() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);
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

        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, mbm, mm);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, new Matrix(1, 1),
                    mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, new Matrix(3, 3),
                    mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, mbm, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, mbm, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor102() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);
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

        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, mbm, mm, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, new Matrix(1, 1),
                    mm, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, new Matrix(3, 3),
                    mm, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, mbm,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, mbm,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor103() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);
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

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, mbm, mm);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true,
                    new Matrix(1, 1), mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true,
                    new Matrix(3, 3), mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, mbm,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, mbm,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor104() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);
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

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, mbm, mm,
                        this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true,
                    new Matrix(1, 1), mm, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true,
                    new Matrix(3, 3), mm, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, mbm,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, mbm,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor105() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);
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

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        magneticModel, mbm, mm);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    magneticModel, new Matrix(1, 1), mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    magneticModel, new Matrix(3, 3), mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    magneticModel, mbm, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    magneticModel, mbm, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor106() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);
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

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        magneticModel, mbm, mm, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    magneticModel, new Matrix(1, 1), mm,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    magneticModel, new Matrix(3, 3), mm,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    magneticModel, mbm, new Matrix(1, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    magneticModel, mbm, new Matrix(3, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor107() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);
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

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, magneticModel, mbm, mm);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, magneticModel,
                    new Matrix(1, 1), mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, magneticModel,
                    new Matrix(3, 3), mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, magneticModel, mbm,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, magneticModel, mbm,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor108() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);
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

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, magneticModel, mbm, mm, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, magneticModel,
                    new Matrix(1, 1), mm, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, magneticModel,
                    new Matrix(3, 3), mm, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, magneticModel, mbm,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, magneticModel, mbm,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor109() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);
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

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, magneticModel, mbm, mm);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, magneticModel,
                    new Matrix(1, 1), mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, magneticModel,
                    new Matrix(3, 3), mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, magneticModel, mbm,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, magneticModel, mbm,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor110() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);
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

        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        true, magneticModel, mbm, mm,
                        this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, magneticModel,
                    new Matrix(1, 1), mm, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, magneticModel,
                    new Matrix(3, 3), mm, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, magneticModel, mbm,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    true, magneticModel, mbm,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor111() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);
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

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel,
                        mbm, mm);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, magneticModel,
                    new Matrix(1, 1), mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, magneticModel,
                    new Matrix(3, 3), mm);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, magneticModel,
                    mbm, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, magneticModel,
                    mbm, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor112() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];
        final Matrix mbm = Matrix.newFromArray(mb);
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

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel,
                        mbm, mm, this);

        // check default values
        assertEquals(calibrator.getInitialHardIronX(), hardIronX,
                0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY,
                0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ,
                0.0);
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
        assertArrayEquals(calibrator.getInitialHardIron(), mb,
                0.0);
        final double[] initialHardIron = new double[3];
        calibrator.getInitialHardIron(initialHardIron);
        assertArrayEquals(initialHardIron, mb, 0.0);
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                Matrix.newFromArray(mb));
        final Matrix initialHardIronMatrix = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(initialHardIronMatrix);
        assertEquals(initialHardIronMatrix, Matrix.newFromArray(mb));
        assertEquals(calibrator.getInitialMm(), mm);
        final Matrix initialMm = new Matrix(3, 3);
        calibrator.getInitialMm(initialMm);
        assertEquals(initialMm, mm);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, magneticModel,
                    new Matrix(1, 1), mm, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, magneticModel,
                    new Matrix(3, 3), mm, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, magneticModel, mbm,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                    measurements, true, magneticModel, mbm,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testGetSetInitialHardIronX() throws LockedException {
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];

        calibrator.setInitialHardIronX(hardIronX);

        // check
        assertEquals(calibrator.getInitialHardIronX(), hardIronX, 0.0);
    }

    @Test
    public void testGetSetInitialHardIronY() throws LockedException {
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronY = mb[1];

        calibrator.setInitialHardIronY(hardIronY);

        // check
        assertEquals(calibrator.getInitialHardIronY(), hardIronY, 0.0);
    }

    @Test
    public void testGetSetInitialHardIronZ() throws LockedException {
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronZ = mb[2];

        calibrator.setInitialHardIronZ(hardIronZ);

        // check
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ, 0.0);
    }

    @Test
    public void testSetInitialHardIron() throws LockedException {
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getInitialHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] mb = generateHardIron(randomizer);
        final double hardIronX = mb[0];
        final double hardIronY = mb[1];
        final double hardIronZ = mb[2];

        calibrator.setInitialHardIron(hardIronX, hardIronY, hardIronZ);

        // check
        assertEquals(calibrator.getInitialHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getInitialHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getInitialHardIronZ(), hardIronZ, 0.0);
    }

    @Test
    public void testGetSetInitialSx() throws LockedException {
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

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
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

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
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

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
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

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
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

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
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

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
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

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
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

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
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

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
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

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
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

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
    public void testSetInitialScalingFactorsAndCrossCouplingErrors() throws LockedException {
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

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
    public void testGetInitialHardIronAsArray() throws LockedException {
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertArrayEquals(calibrator.getInitialHardIron(),
                new double[3], 0.0);
        final double[] result1 = new double[3];
        calibrator.getInitialHardIron(result1);
        assertArrayEquals(result1, new double[3], 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] bm = generateHardIron(randomizer);
        calibrator.setInitialHardIron(bm);

        // check
        assertArrayEquals(calibrator.getInitialHardIron(), bm, 0.0);
        final double[] result2 = new double[3];
        calibrator.getInitialHardIron(result2);
        assertArrayEquals(result2, bm, 0.0);

        // Force IllegalArgumentException
        try {
            calibrator.getInitialHardIron(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setInitialHardIron(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetInitialHardIronAsMatrix() throws LockedException,
            WrongSizeException {
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getInitialHardIronAsMatrix(),
                new Matrix(3, 1));
        final Matrix result1 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(result1);
        assertEquals(result1, new Matrix(3, 1));

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] bm = generateHardIron(randomizer);
        final Matrix b = Matrix.newFromArray(bm);
        calibrator.setInitialHardIron(b);

        // check
        assertEquals(calibrator.getInitialHardIronAsMatrix(), b);
        final Matrix result2 = new Matrix(3, 1);
        calibrator.getInitialHardIronAsMatrix(result2);
        assertEquals(result2, b);

        // Force IllegalArgumentException
        try {
            calibrator.getInitialHardIronAsMatrix(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.getInitialHardIronAsMatrix(new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setInitialHardIron(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setInitialHardIron(new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetInitialMm() throws WrongSizeException,
            LockedException {
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

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
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

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
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testGetListener() throws LockedException {
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testIsReady() throws LockedException, IOException, InvalidSourceAndDestinationFrameTypeException {
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

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
                        hardIron, softIron, KnownFrameMagnetometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                        wmmEstimator, randomizer, null);

        calibrator.setMeasurements(measurements);

        // check
        assertTrue(calibrator.isReady());
    }

    @Test
    public void testGetSetMagneticModel() throws LockedException {
        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getMagneticModel());

        // set new value
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        calibrator.setMagneticModel(magneticModel);

        // check
        assertSame(calibrator.getMagneticModel(), magneticModel);
    }

    @Test
    public void testCalibrateMultipleOrientationsForGeneralCaseWithMinimumMeasuresNoNoiseAndNoMagneticModel()
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException, WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                generateMeasurementsMultipleOrientationswithSamePosition(
                        hardIron.getBuffer(), mm,
                        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                        wmmEstimator, randomizer, null);

        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, false, hardIron, mm,
                        this);

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

        final Matrix estimatedHardIron = calibrator
                .getEstimatedHardIronAsMatrix();
        final Matrix estimatedMm = calibrator.getEstimatedMm();

        assertTrue(hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR));
        assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);
    }

    @Test
    public void testCalibrateMultipleOrientationsForGeneralCaseWithMinimumMeasuresNoNoiseAndMagneticModel()
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException, WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final WorldMagneticModel model = wmmEstimator.getModel();

        final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                generateMeasurementsMultipleOrientationswithSamePosition(
                        hardIron.getBuffer(), mm,
                        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                        wmmEstimator, randomizer, null);

        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, false, model, hardIron, mm,
                        this);

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

        final Matrix estimatedHardIron = calibrator
                .getEstimatedHardIronAsMatrix();
        final Matrix estimatedMm = calibrator.getEstimatedMm();

        assertTrue(hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR));
        assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);
    }

    @Test
    public void testCalibrateMultipleOrientationsForGeneralCaseWithNoiseLargeNumberOfMeasurements()
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException, WrongSizeException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                    new WMMEarthMagneticFluxDensityEstimator();

            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, MAGNETOMETER_NOISE_STD);

            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                    generateMeasurementsMultipleOrientationswithSamePosition(
                            hardIron.getBuffer(), mm,
                            LARGE_MEASUREMENT_NUMBER,
                            wmmEstimator, randomizer, noiseRandomizer);

            final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                    new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                            measurements, false, hardIron, mm,
                            this);

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

            final Matrix estimatedHardIron = calibrator
                    .getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(hardIron.equals(estimatedHardIron,
                    LARGE_ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultipleOrientationsForGeneralCaseWithNoiseSmallNumberOfMeasurements()
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException, WrongSizeException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                    new WMMEarthMagneticFluxDensityEstimator();

            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, MAGNETOMETER_NOISE_STD);

            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                    generateMeasurementsMultipleOrientationswithSamePosition(
                            hardIron.getBuffer(), mm,
                            SMALL_MEASUREMENT_NUMBER,
                            wmmEstimator, randomizer, noiseRandomizer);

            final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                    new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                            measurements, false, hardIron, mm,
                            this);

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

            final Matrix estimatedHardIron = calibrator
                    .getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(hardIron.equals(estimatedHardIron,
                    VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultiplePositionsForGeneralCaseWithMinimumMeasuresAndNoNoise()
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException, WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                    generateMeasurementsMultiplePositionsWithSameOrientation(
                            hardIron.getBuffer(), mm,
                            wmmEstimator, randomizer);

            final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                    new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                            measurements, false, hardIron, mm,
                            this);

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

            final Matrix estimatedHardIron = calibrator
                    .getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultipleOrientationsForCommonAxisCaseWithMinimumMeasuresNoNoiseAndNoMagneticModel()
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException, WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix mm = generateSoftIronCommonAxis();
        assertNotNull(mm);

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                generateMeasurementsMultipleOrientationswithSamePosition(
                        hardIron.getBuffer(), mm,
                        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                        wmmEstimator, randomizer, null);

        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, hardIron, mm,
                        this);

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

        final Matrix estimatedHardIron = calibrator
                .getEstimatedHardIronAsMatrix();
        final Matrix estimatedMm = calibrator.getEstimatedMm();

        assertTrue(hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR));
        assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);
    }

    @Test
    public void testCalibrateMultipleOrientationsForCommonAxisCaseWithMinimumMeasuresNoNoiseAndMagneticModel()
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException, WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final WorldMagneticModel model = wmmEstimator.getModel();

        final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
        final Matrix mm = generateSoftIronCommonAxis();
        assertNotNull(mm);

        final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                generateMeasurementsMultipleOrientationswithSamePosition(
                        hardIron.getBuffer(), mm,
                        KnownFrameMagnetometerNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                        wmmEstimator, randomizer, null);

        final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                        measurements, true, model, hardIron,
                        mm, this);

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

        final Matrix estimatedHardIron = calibrator
                .getEstimatedHardIronAsMatrix();
        final Matrix estimatedMm = calibrator.getEstimatedMm();

        assertTrue(hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR));
        assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);
    }

    @Test
    public void testCalibrateMultipleOrientationsForCommonAxisCaseWithNoiseLargeNumberOfMeasurements()
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException, WrongSizeException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                    new WMMEarthMagneticFluxDensityEstimator();

            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, MAGNETOMETER_NOISE_STD);

            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                    generateMeasurementsMultipleOrientationswithSamePosition(
                            hardIron.getBuffer(), mm,
                            LARGE_MEASUREMENT_NUMBER,
                            wmmEstimator, randomizer, noiseRandomizer);

            final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                    new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                            measurements, true, hardIron, mm,
                            this);

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

            final Matrix estimatedHardIron = calibrator
                    .getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(hardIron.equals(estimatedHardIron,
                    LARGE_ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultipleOrientationsForCommonAxisCaseWithNoiseSmallNumberOfMeasurements()
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException, WrongSizeException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                    new WMMEarthMagneticFluxDensityEstimator();

            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, MAGNETOMETER_NOISE_STD);

            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                    generateMeasurementsMultipleOrientationswithSamePosition(
                            hardIron.getBuffer(), mm,
                            SMALL_MEASUREMENT_NUMBER,
                            wmmEstimator, randomizer, noiseRandomizer);

            final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                    new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                            measurements, true, hardIron, mm,
                            this);

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

            final Matrix estimatedHardIron = calibrator
                    .getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(hardIron.equals(estimatedHardIron,
                    VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultiplePositionsForCommonAxisCaseWithMinimumMeasuresAndNoNoise()
            throws IOException, InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            CalibrationException, WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final List<StandardDeviationFrameBodyMagneticFluxDensity> measurements =
                    generateMeasurementsMultiplePositionsWithSameOrientation(
                            hardIron.getBuffer(), mm,
                            wmmEstimator, randomizer);

            final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator =
                    new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator(
                            measurements, true, hardIron, mm,
                            this);

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

            final Matrix estimatedHardIron = calibrator
                    .getEstimatedHardIronAsMatrix();
            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedHardIron, estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        mCalibrateStart++;
    }

    @Override
    public void onCalibrateEnd(KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        mCalibrateEnd++;
    }

    private void reset() {
        mCalibrateStart = 0;
        mCalibrateEnd = 0;
    }

    private void checkLocked(
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
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
            calibrator.setInitialScalingFactors(
                    0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialCrossCouplingErrors(
                    0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0);
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
            calibrator.calibrate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
    }

    private void assertEstimatedResult(
            final Matrix hardIron, final Matrix mm,
            final KnownFrameMagnetometerNonLinearLeastSquaresCalibrator calibrator)
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

    private static List<StandardDeviationFrameBodyMagneticFluxDensity>
    generateMeasurementsMultipleOrientationswithSamePosition(
            final double[] hardIron, final Matrix softIron,
            final int numberOfMeasurements,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final UniformRandomizer randomizer,
            final GaussianRandomizer noiseRandomizer)
            throws InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition position = createPosition(randomizer);
        final List<StandardDeviationFrameBodyMagneticFluxDensity> result =
                new ArrayList<>();
        for (int i = 0; i < numberOfMeasurements; i++) {
            result.add(generateMeasureAtPosition(hardIron, softIron,
                    wmmEstimator, randomizer, noiseRandomizer, position));
        }
        return result;
    }

    private static List<StandardDeviationFrameBodyMagneticFluxDensity>
    generateMeasurementsMultiplePositionsWithSameOrientation(
            final double[] hardIron, final Matrix softIron,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final UniformRandomizer randomizer)
            throws InvalidSourceAndDestinationFrameTypeException {
        final CoordinateTransformation cnb = new CoordinateTransformation(
                FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final List<StandardDeviationFrameBodyMagneticFluxDensity> result =
                new ArrayList<>();
        for (int i = 0; i < KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {
            result.add(generateMeasureAtOrientation(hardIron, softIron,
                    wmmEstimator, randomizer, cnb));
        }
        return result;
    }

    private static StandardDeviationFrameBodyMagneticFluxDensity generateMeasureAtOrientation(
            final double[] hardIron, final Matrix softIron,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final UniformRandomizer randomizer,
            final CoordinateTransformation cnb)
            throws InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition position = createPosition(randomizer);
        return generateMeasure(hardIron, softIron, wmmEstimator,
                randomizer, null, position, cnb);
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
