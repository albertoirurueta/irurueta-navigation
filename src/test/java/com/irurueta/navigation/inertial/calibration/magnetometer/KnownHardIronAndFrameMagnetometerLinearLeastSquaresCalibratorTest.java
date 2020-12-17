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
import com.irurueta.navigation.inertial.calibration.FrameBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
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

public class KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibratorTest implements
        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibratorListener {

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

    private static final double MAGNETOMETER_NOISE_STD = 200e-9;

    private static final double ABSOLUTE_ERROR = 1e-9;
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
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), new double[3],
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, new double[3], 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, new Matrix(3, 1));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
    }

    @Test
    public void testConstructor2() throws WrongSizeException {
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        this);

        // check default values
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), new double[3],
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, new double[3], 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, new Matrix(3, 1));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
    }

    @Test
    public void testConstructor3() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements);

        // check default values
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), new double[3],
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, new double[3], 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, new Matrix(3, 1));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
    }

    @Test
    public void testConstructor4() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), new double[3],
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, new double[3], 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, new Matrix(3, 1));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
    }

    @Test
    public void testConstructor5() throws WrongSizeException {
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        true);

        // check default values
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), new double[3],
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, new double[3], 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, new Matrix(3, 1));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
    }

    @Test
    public void testConstructor6() throws WrongSizeException {
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        true, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), new double[3],
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, new double[3], 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, new Matrix(3, 1));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
    }

    @Test
    public void testConstructor7() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, true);

        // check default values
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), new double[3],
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, new double[3], 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, new Matrix(3, 1));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
    }

    @Test
    public void testConstructor8() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, true, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), new double[3],
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, new double[3], 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, new Matrix(3, 1));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
    }

    @Test
    public void testConstructor9() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        magneticModel);

        // check default values
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), new double[3],
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, new double[3], 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, new Matrix(3, 1));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
    }

    @Test
    public void testConstructor10() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        magneticModel, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), new double[3],
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, new double[3], 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, new Matrix(3, 1));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
    }

    @Test
    public void testConstructor11() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, magneticModel);

        // check default values
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), new double[3],
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, new double[3], 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, new Matrix(3, 1));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
    }

    @Test
    public void testConstructor12() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, magneticModel, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), new double[3],
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, new double[3], 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, new Matrix(3, 1));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
    }

    @Test
    public void testConstructor13() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        true, magneticModel);

        // check default values
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), new double[3],
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, new double[3], 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, new Matrix(3, 1));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
    }

    @Test
    public void testConstructor14() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        true, magneticModel, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), new double[3],
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, new double[3], 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, new Matrix(3, 1));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
    }

    @Test
    public void testConstructor15() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel);

        // check default values
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), new double[3],
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, new double[3], 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, new Matrix(3, 1));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
    }

    @Test
    public void testConstructor16() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel,
                        this);

        // check default values
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), new double[3],
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, new double[3], 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, new Matrix(3, 1));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
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
    }

    @Test
    public void testConstructor17() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        hardIronX, hardIronY, hardIronZ);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
    }

    @Test
    public void testConstructor18() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        hardIronX, hardIronY, hardIronZ, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
    }

    @Test
    public void testConstructor19() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, hardIronX, hardIronY, hardIronZ);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
    }

    @Test
    public void testConstructor20() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, hardIronX, hardIronY, hardIronZ,
                        this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
    }

    @Test
    public void testConstructor21() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        true, hardIronX, hardIronY, hardIronZ);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
    }

    @Test
    public void testConstructor22() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        true, hardIronX, hardIronY, hardIronZ,
                        this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
    }

    @Test
    public void testConstructor23() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, true, hardIronX,
                        hardIronY, hardIronZ);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
    }

    @Test
    public void testConstructor24() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, true, hardIronX,
                        hardIronY, hardIronZ, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
    }

    @Test
    public void testConstructor25() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        magneticModel, hardIronX, hardIronY, hardIronZ);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
    }

    @Test
    public void testConstructor27() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        magneticModel, hardIronX, hardIronY, hardIronZ);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
    }

    @Test
    public void testConstructor28() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        magneticModel, hardIronX, hardIronY, hardIronZ,
                        this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
    }

    @Test
    public void testConstructor29() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, magneticModel, hardIronX, hardIronY,
                        hardIronZ);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
    }

    @Test
    public void testConstructor30() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, magneticModel, hardIronX, hardIronY,
                        hardIronZ, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
    }

    @Test
    public void testConstructor31() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        true, magneticModel, hardIronX,
                        hardIronY, hardIronZ);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
    }

    @Test
    public void testConstructor32() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        true, magneticModel, hardIronX,
                        hardIronY, hardIronZ, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
    }

    @Test
    public void testConstructor33() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel,
                        hardIronX, hardIronY, hardIronZ);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
    }

    @Test
    public void testConstructor34() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel,
                        hardIronX, hardIronY, hardIronZ, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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
    }

    @Test
    public void testConstructor35() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        hardIron);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor36() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        hardIron, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor37() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, hardIron);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor38() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, hardIron, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor39() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        true, hardIron);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    true, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor40() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        true, hardIron, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    true, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor41() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, true, hardIron);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, true, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor42() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, true, hardIron,
                        this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, true, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor43() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        magneticModel, hardIron);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    magneticModel, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor44() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        magneticModel, hardIron, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    magneticModel, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor45() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, magneticModel, hardIron);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, magneticModel, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor46() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, magneticModel, hardIron, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, magneticModel, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor47() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        true, magneticModel, hardIron);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    true, magneticModel, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor48() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        true, magneticModel, hardIron,
                        this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    true, magneticModel, new double[1],
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor49() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel,
                        hardIron);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, true, magneticModel,
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor50() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();

        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel,
                        hardIron, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, Matrix.newFromArray(hardIron));
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, true, magneticModel,
                    new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor51() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final Matrix hardIronMatrix = Matrix.newFromArray(hardIron);

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        hardIronMatrix);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor52() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final Matrix hardIronMatrix = Matrix.newFromArray(hardIron);

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        hardIronMatrix, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    new Matrix(1, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    new Matrix(3, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor53() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final Matrix hardIronMatrix = Matrix.newFromArray(hardIron);

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, hardIronMatrix);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor54() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final Matrix hardIronMatrix = Matrix.newFromArray(hardIron);

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, hardIronMatrix, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, new Matrix(1, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, new Matrix(3, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor55() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final Matrix hardIronMatrix = Matrix.newFromArray(hardIron);

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        true, hardIronMatrix);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    true, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    true, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor56() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final Matrix hardIronMatrix = Matrix.newFromArray(hardIron);

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        true, hardIronMatrix, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    true, new Matrix(1, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    true, new Matrix(3, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor57() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final Matrix hardIronMatrix = Matrix.newFromArray(hardIron);

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, true, hardIronMatrix);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, true,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, true,
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor58() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final Matrix hardIronMatrix = Matrix.newFromArray(hardIron);

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, true, hardIronMatrix,
                        this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertNull(calibrator.getMagneticModel());
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, true,
                    new Matrix(1, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, true,
                    new Matrix(3, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor59() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final Matrix hardIronMatrix = Matrix.newFromArray(hardIron);

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        magneticModel, hardIronMatrix);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    magneticModel, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    magneticModel, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor60() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final Matrix hardIronMatrix = Matrix.newFromArray(hardIron);

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        magneticModel, hardIronMatrix, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    magneticModel, new Matrix(1, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    magneticModel, new Matrix(3, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor61() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final Matrix hardIronMatrix = Matrix.newFromArray(hardIron);

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, magneticModel, hardIronMatrix);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, magneticModel,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, magneticModel,
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor62() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final Matrix hardIronMatrix = Matrix.newFromArray(hardIron);

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, magneticModel, hardIronMatrix,
                        this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertFalse(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, magneticModel,
                    new Matrix(1, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, magneticModel,
                    new Matrix(3, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor63() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final Matrix hardIronMatrix = Matrix.newFromArray(hardIron);

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        true, magneticModel, hardIronMatrix);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    true, magneticModel,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    true, magneticModel,
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor64() throws WrongSizeException {
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final Matrix hardIronMatrix = Matrix.newFromArray(hardIron);

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        true, magneticModel, hardIronMatrix,
                        this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertNull(calibrator.getMeasurements());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    true, magneticModel,
                    new Matrix(1, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    true, magneticModel,
                    new Matrix(3, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor65() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final Matrix hardIronMatrix = Matrix.newFromArray(hardIron);

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel,
                        hardIronMatrix);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, true, magneticModel,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, true, magneticModel,
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor66() throws WrongSizeException {
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        final Matrix hardIronMatrix = Matrix.newFromArray(hardIron);

        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, true, magneticModel,
                        hardIronMatrix, this);

        // check default values
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertSame(calibrator.getMagneticModel(), magneticModel);
        assertArrayEquals(calibrator.getHardIron(), hardIron,
                0.0);
        final double[] hardIron1 = new double[3];
        calibrator.getHardIron(hardIron1);
        assertArrayEquals(hardIron1, hardIron, 0.0);
        final Matrix hardIron2 = calibrator.getHardIronMatrix();
        assertEquals(hardIron2, hardIronMatrix);
        final Matrix hardIron3 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron3);
        assertEquals(hardIron2, hardIron3);
        MagneticFluxDensity b1 = calibrator.getHardIronXAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronX, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensity b2 = new MagneticFluxDensity(1.0,
                MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronXAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronYAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronY, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronYAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        b1 = calibrator.getHardIronZAsMagneticFluxDensity();
        assertEquals(b1.getValue().doubleValue(), hardIronZ, 0.0);
        assertEquals(b1.getUnit(), MagneticFluxDensityUnit.TESLA);
        calibrator.getHardIronZAsMagneticFluxDensity(b2);
        assertEquals(b1, b2);
        final MagneticFluxDensityTriad hardIronTriad1 = calibrator.getHardIronAsTriad();
        assertEquals(hardIronTriad1.getValueX(), hardIronX, 0.0);
        assertEquals(hardIronTriad1.getValueY(), hardIronY, 0.0);
        assertEquals(hardIronTriad1.getValueZ(), hardIronZ, 0.0);
        assertEquals(hardIronTriad1.getUnit(), MagneticFluxDensityUnit.TESLA);
        final MagneticFluxDensityTriad hardIronTriad2 = new MagneticFluxDensityTriad();
        calibrator.getHardIronAsTriad(hardIronTriad2);
        assertEquals(hardIronTriad1, hardIronTriad2);
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

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, true, magneticModel,
                    new Matrix(1, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                    measurements, true, magneticModel,
                    new Matrix(3, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testGetSetHardIronX() throws LockedException {
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];

        calibrator.setHardIronX(hardIronX);

        // check
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
    }

    @Test
    public void testGetSetHardIronY() throws LockedException {
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronY = hardIron[1];

        calibrator.setHardIronY(hardIronY);

        // check
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
    }

    @Test
    public void testGetSetHardIronZ() throws LockedException {
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronZ = hardIron[2];

        calibrator.setHardIronZ(hardIronZ);

        // check
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
    }

    @Test
    public void testGetSetHardIronXAsMagneticFluxDensity()
            throws LockedException {
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

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
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

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
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

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
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getHardIronX(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronY(), 0.0, 0.0);
        assertEquals(calibrator.getHardIronZ(), 0.0, 0.0);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final double[] hardIron = generateHardIron(randomizer);
        final double hardIronX = hardIron[0];
        final double hardIronY = hardIron[1];
        final double hardIronZ = hardIron[2];

        calibrator.setHardIronCoordinates(hardIronX, hardIronY, hardIronZ);

        // check
        assertEquals(calibrator.getHardIronX(), hardIronX, 0.0);
        assertEquals(calibrator.getHardIronY(), hardIronY, 0.0);
        assertEquals(calibrator.getHardIronZ(), hardIronZ, 0.0);
    }

    @Test
    public void testSetHardIronCoordinates2() throws LockedException {
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

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
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

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
    public void testGetSetMeasurements() throws LockedException {
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getMeasurements());

        // set new value
        final List<FrameBodyMagneticFluxDensity> measurements =
                Collections.emptyList();
        calibrator.setMeasurements(measurements);

        // check
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testIsSetCommonAxisUsed() throws LockedException {
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testGetSetMagneticModel() throws LockedException {
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getMagneticModel());

        // set new value
        final WorldMagneticModel magneticModel = new WorldMagneticModel();
        calibrator.setMagneticModel(magneticModel);

        // check
        assertSame(calibrator.getMagneticModel(), magneticModel);
    }

    @Test
    public void testGetSetHardIron() throws LockedException {
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        final double[] hardIron1 = calibrator.getHardIron();
        final double[] hardIron2 = new double[3];
        calibrator.getHardIron(hardIron2);

        assertArrayEquals(hardIron1, new double[3], 0.0);
        assertArrayEquals(hardIron1, hardIron2, 0.0);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double[] hardIron3 = generateHardIron(randomizer);

        calibrator.setHardIron(hardIron3);

        // check
        final double[] hardIron4 = calibrator.getHardIron();
        final double[] hardIron5 = new double[3];
        calibrator.getHardIron(hardIron5);

        assertArrayEquals(hardIron3, hardIron4, 0.0);
        assertArrayEquals(hardIron3, hardIron5, 0.0);

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
    public void testGetSetHardIronMatrix() throws WrongSizeException, LockedException {
        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator();

        // check default value
        final Matrix hardIron1 = calibrator.getHardIronMatrix();
        final Matrix hardIron2 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron2);

        assertEquals(hardIron1, new Matrix(3, 1));
        assertEquals(hardIron1, hardIron2);

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(
                new Random());
        final Matrix hardIron3 = Matrix.newFromArray(
                generateHardIron(randomizer));

        calibrator.setHardIron(hardIron3);

        // check
        final Matrix hardIron4 = calibrator.getHardIronMatrix();
        final Matrix hardIron5 = new Matrix(3, 1);
        calibrator.getHardIronMatrix(hardIron5);

        assertEquals(hardIron3, hardIron4);
        assertEquals(hardIron3, hardIron5);
    }

    @Test
    public void testCalibrateMultipleOrientationsForGeneralCaseWithMinimumMeasuresNoNoiseAndNoMagneticModel()
            throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, CalibrationException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final List<FrameBodyMagneticFluxDensity> measurements =
                generateMeasurementsMultipleOrientationswithSamePosition(
                        hardIron, mm,
                        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                        wmmEstimator, randomizer, null);

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, false, hardIron, this);

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

        final Matrix estimatedMm = calibrator.getEstimatedMm();

        assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMm, calibrator);
    }

    @Test
    public void testCalibrateMultipleOrientationsForGeneralCaseWithMinimumMeasuresNoNoiseAndMagneticModel()
            throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, CalibrationException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final WorldMagneticModel model = wmmEstimator.getModel();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix mm = generateSoftIronGeneral();
        assertNotNull(mm);

        final List<FrameBodyMagneticFluxDensity> measurements =
                generateMeasurementsMultipleOrientationswithSamePosition(
                        hardIron, mm,
                        KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                        wmmEstimator, randomizer, null);

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, false, model, hardIron,
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

        final Matrix estimatedMm = calibrator.getEstimatedMm();

        assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMm, calibrator);
    }

    @Test
    public void testCalibrateMultipleOrientationsForGeneralCaseWithNoiseLargeNumberOfMeasurements()
            throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, CalibrationException {

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

            final List<FrameBodyMagneticFluxDensity> measurements =
                    generateMeasurementsMultipleOrientationswithSamePosition(
                            hardIron, mm, LARGE_MEASUREMENT_NUMBER,
                            wmmEstimator, randomizer, noiseRandomizer);

            final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                    new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                            measurements, false, hardIron,
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

            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultipleOrientationsForGeneralCaseWithNoiseSmallNumberOfMeasurements()
            throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, CalibrationException {

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

            final List<FrameBodyMagneticFluxDensity> measurements =
                    generateMeasurementsMultipleOrientationswithSamePosition(
                            hardIron, mm, SMALL_MEASUREMENT_NUMBER,
                            wmmEstimator, randomizer, noiseRandomizer);

            final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                    new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                            measurements, false, hardIron,
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

            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultiplePositionsForGeneralCaseWithMinimumMeasuresAndNoNoise()
            throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, CalibrationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                    new WMMEarthMagneticFluxDensityEstimator();

            final double[] hardIron = generateHardIron(randomizer);
            final Matrix mm = generateSoftIronGeneral();
            assertNotNull(mm);

            final List<FrameBodyMagneticFluxDensity> measurements =
                    generateMeasurementsMultiplePositionsWithSameOrientation(
                            hardIron, mm,
                            wmmEstimator, randomizer);

            final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                    new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                            measurements, false, hardIron,
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

            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultipleOrientationsForCommonAxisCaseWithMinimumMeasuresNoNoiseAndNoMagneticModel()
            throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, CalibrationException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix mm = generateSoftIronCommonAxis();
        assertNotNull(mm);

        final List<FrameBodyMagneticFluxDensity> measurements =
                generateMeasurementsMultipleOrientationswithSamePosition(
                        hardIron, mm,
                        KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                        wmmEstimator, randomizer, null);

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, true, hardIron,
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

        final Matrix estimatedMm = calibrator.getEstimatedMm();

        assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMm, calibrator);
    }

    @Test
    public void testCalibrateMultipleOrientationsForCommonAxisCaseWithMinimumMeasuresNoNoiseAndMagneticModel()
            throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, CalibrationException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                new WMMEarthMagneticFluxDensityEstimator();
        final WorldMagneticModel model = wmmEstimator.getModel();

        final double[] hardIron = generateHardIron(randomizer);
        final Matrix mm = generateSoftIronCommonAxis();
        assertNotNull(mm);

        final List<FrameBodyMagneticFluxDensity> measurements =
                generateMeasurementsMultipleOrientationswithSamePosition(
                        hardIron, mm,
                        KnownFrameMagnetometerLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                        wmmEstimator, randomizer, null);

        final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                        measurements, true, model, hardIron,
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

        final Matrix estimatedMm = calibrator.getEstimatedMm();

        assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMm, calibrator);
    }

    @Test
    public void testCalibrateMultipleOrientationsForCommonAxisCaseWithNoiseLargeNumberOfMeasurements()
            throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, CalibrationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                    new WMMEarthMagneticFluxDensityEstimator();

            final double[] hardIron = generateHardIron(randomizer);
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, MAGNETOMETER_NOISE_STD);

            final List<FrameBodyMagneticFluxDensity> measurements =
                    generateMeasurementsMultipleOrientationswithSamePosition(
                            hardIron, mm, LARGE_MEASUREMENT_NUMBER,
                            wmmEstimator, randomizer, noiseRandomizer);

            final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                    new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                            measurements, true, hardIron,
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

            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm,
                    VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultipleOrientationsForCommonAxisCaseWithNoiseSmallNumberOfMeasurements()
            throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, CalibrationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                    new WMMEarthMagneticFluxDensityEstimator();

            final double[] hardIron = generateHardIron(randomizer);
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final GaussianRandomizer noiseRandomizer = new GaussianRandomizer(
                    new Random(), 0.0, MAGNETOMETER_NOISE_STD);

            final List<FrameBodyMagneticFluxDensity> measurements =
                    generateMeasurementsMultipleOrientationswithSamePosition(
                            hardIron, mm, SMALL_MEASUREMENT_NUMBER,
                            wmmEstimator, randomizer, noiseRandomizer);

            final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                    new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                            measurements, true, hardIron,
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

            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultiplePositionsForCommonAxisCaseWithMinimumMeasuresAndNoNoise()
            throws IOException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, CalibrationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator =
                    new WMMEarthMagneticFluxDensityEstimator();

            final double[] hardIron = generateHardIron(randomizer);
            final Matrix mm = generateSoftIronCommonAxis();
            assertNotNull(mm);

            final List<FrameBodyMagneticFluxDensity> measurements =
                    generateMeasurementsMultiplePositionsWithSameOrientation(
                            hardIron, mm,
                            wmmEstimator, randomizer);

            final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator =
                    new KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator(
                            measurements, true, hardIron,
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

            final Matrix estimatedMm = calibrator.getEstimatedMm();

            if (!mm.equals(estimatedMm, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mm.equals(estimatedMm, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMm, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(
            final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        mCalibrateStart++;
    }

    @Override
    public void onCalibrateEnd(
            final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        mCalibrateEnd++;
    }

    private void reset() {
        mCalibrateStart = 0;
        mCalibrateEnd = 0;
    }

    private void checkLocked(
            final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
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
            calibrator.setHardIronCoordinates(0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setHardIronCoordinates(null, null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setHardIron((MagneticFluxDensityTriad) null);
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
            calibrator.calibrate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
    }

    private void assertEstimatedResult(
            final Matrix mm,
            final KnownHardIronAndFrameMagnetometerLinearLeastSquaresCalibrator calibrator) {

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
