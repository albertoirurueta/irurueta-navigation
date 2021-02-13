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
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.FrameBodyKinematics;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibratorTest implements
        KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibratorListener {

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
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(new double[3], bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, new Matrix(3, 1));
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor2() throws WrongSizeException {
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(new double[3], bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, new Matrix(3, 1));
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor3() throws WrongSizeException {
        final Collection<FrameBodyKinematics> measurements = Collections.emptyList();
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(new double[3], bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, new Matrix(3, 1));
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor4() throws WrongSizeException {
        final Collection<FrameBodyKinematics> measurements = Collections.emptyList();
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        measurements, this);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(new double[3], bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, new Matrix(3, 1));
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor5() throws WrongSizeException {
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        true);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(new double[3], bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, new Matrix(3, 1));
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor6() throws WrongSizeException {
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        true, this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(new double[3], bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, new Matrix(3, 1));
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor7() throws WrongSizeException {
        final Collection<FrameBodyKinematics> measurements = Collections.emptyList();
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        measurements, true);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(new double[3], bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, new Matrix(3, 1));
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor8() throws WrongSizeException {
        final Collection<FrameBodyKinematics> measurements = Collections.emptyList();
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        measurements, true, this);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(new double[3], bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, new Matrix(3, 1));
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor9() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double biasX = bg.getElementAtIndex(0);
        final double biasY = bg.getElementAtIndex(1);
        final double biasZ = bg.getElementAtIndex(2);
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        biasX, biasY, biasZ);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor10() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double biasX = bg.getElementAtIndex(0);
        final double biasY = bg.getElementAtIndex(1);
        final double biasZ = bg.getElementAtIndex(2);
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        biasX, biasY, biasZ, this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor11() throws WrongSizeException {
        final Collection<FrameBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final double biasX = bg.getElementAtIndex(0);
        final double biasY = bg.getElementAtIndex(1);
        final double biasZ = bg.getElementAtIndex(2);
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        measurements, biasX, biasY, biasZ);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor12() throws WrongSizeException {
        final Collection<FrameBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final double biasX = bg.getElementAtIndex(0);
        final double biasY = bg.getElementAtIndex(1);
        final double biasZ = bg.getElementAtIndex(2);
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        measurements, biasX, biasY, biasZ, this);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor13() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double biasX = bg.getElementAtIndex(0);
        final double biasY = bg.getElementAtIndex(1);
        final double biasZ = bg.getElementAtIndex(2);
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        biasX, biasY, biasZ, true);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor14() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double biasX = bg.getElementAtIndex(0);
        final double biasY = bg.getElementAtIndex(1);
        final double biasZ = bg.getElementAtIndex(2);
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        biasX, biasY, biasZ, true, this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor15() throws WrongSizeException {
        final Collection<FrameBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final double biasX = bg.getElementAtIndex(0);
        final double biasY = bg.getElementAtIndex(1);
        final double biasZ = bg.getElementAtIndex(2);
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        measurements, biasX, biasY, biasZ, true);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor16() throws WrongSizeException {
        final Collection<FrameBodyKinematics> measurements = Collections.emptyList();
        final Matrix bg = generateBg();
        final double biasX = bg.getElementAtIndex(0);
        final double biasY = bg.getElementAtIndex(1);
        final double biasZ = bg.getElementAtIndex(2);
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        measurements, biasX, biasY, biasZ, true,
                        this);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor17() throws WrongSizeException {
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
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        bx, by, bz);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
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
    }

    @Test
    public void testConstructor18() throws WrongSizeException {
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
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        bx, by, bz, this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor19() throws WrongSizeException {
        final Collection<FrameBodyKinematics> measurements = Collections.emptyList();
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
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        measurements, bx, by, bz);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor20() throws WrongSizeException {
        final Collection<FrameBodyKinematics> measurements = Collections.emptyList();
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
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        measurements, bx, by, bz, this);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor21() throws WrongSizeException {
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
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        bx, by, bz, true);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor22() throws WrongSizeException {
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
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        bx, by, bz, true, this);

        // check default values
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor23() throws WrongSizeException {
        final Collection<FrameBodyKinematics> measurements = Collections.emptyList();
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
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        measurements, bx, by, bz, true);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testConstructor24() throws WrongSizeException {
        final Collection<FrameBodyKinematics> measurements = Collections.emptyList();
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
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(
                        measurements, bx, by, bz, true, this);

        // check default values
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), biasX, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx2);
        assertEquals(bx1, bx2);
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), biasY, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed by2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by2);
        assertEquals(by1, by2);
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), biasZ, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        AngularSpeed bz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz2);
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bg.getBuffer(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), biasX, 0.0);
        assertEquals(biasTriad1.getValueY(), biasY, 0.0);
        assertEquals(biasTriad1.getValueZ(), biasZ, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(biasTriad2);
        assertEquals(biasTriad1, biasTriad2);
        assertEquals(KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
    }

    @Test
    public void testGetSetMeasurements() throws LockedException {
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

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
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testGetSetBiasX() throws LockedException {
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double biasX = bg.getElementAtIndex(0);

        calibrator.setBiasX(biasX);

        // check
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
    }

    @Test
    public void testGetSetBiasY() throws LockedException {
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double biasY = bg.getElementAtIndex(1);

        calibrator.setBiasY(biasY);

        // check
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
    }

    @Test
    public void testGetSetBiasZ() throws LockedException {
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double biasZ = bg.getElementAtIndex(2);

        calibrator.setBiasZ(biasZ);

        // check
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
    }

    @Test
    public void testGetSetBiasAngularSpeedX() throws LockedException {
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        final AngularSpeed bx1 = calibrator.getBiasAngularSpeedX();
        assertEquals(bx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

        // set new value
        final Matrix bg = generateBg();
        final double biasX = bg.getElementAtIndex(0);
        final AngularSpeed bx2 = new AngularSpeed(biasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setBiasX(bx2);

        // check
        final AngularSpeed bx3 = calibrator.getBiasAngularSpeedX();
        final AngularSpeed bx4 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(bx4);

        assertEquals(bx2, bx3);
        assertEquals(bx2, bx4);
    }

    @Test
    public void testGetSetBiasAngularSpeedY() throws LockedException {
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        final AngularSpeed by1 = calibrator.getBiasAngularSpeedY();
        assertEquals(by1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(by1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

        // set new value
        final Matrix bg = generateBg();
        final double biasY = bg.getElementAtIndex(1);
        final AngularSpeed by2 = new AngularSpeed(biasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setBiasY(by2);

        // check
        final AngularSpeed by3 = calibrator.getBiasAngularSpeedY();
        final AngularSpeed by4 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(by4);

        assertEquals(by2, by3);
        assertEquals(by2, by4);
    }

    @Test
    public void testGetSetBiasAngularSpeedZ() throws LockedException {
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        final AngularSpeed bz1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(bz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

        // set new value
        final Matrix bg = generateBg();
        final double biasZ = bg.getElementAtIndex(2);
        final AngularSpeed bz2 = new AngularSpeed(biasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setBiasZ(bz2);

        // check
        final AngularSpeed bz3 = calibrator.getBiasAngularSpeedZ();
        final AngularSpeed bz4 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(bz4);

        assertEquals(bz2, bz3);
        assertEquals(bz2, bz4);
    }

    @Test
    public void testSetBiasCoordinates1() throws LockedException {
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double biasX = bg.getElementAtIndex(0);
        final double biasY = bg.getElementAtIndex(1);
        final double biasZ = bg.getElementAtIndex(2);

        calibrator.setBiasCoordinates(biasX, biasY, biasZ);

        // check
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
    }

    @Test
    public void testSetBiasCoordinates2() throws LockedException {
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);

        // set new value
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

        calibrator.setBiasCoordinates(bx, by, bz);

        // check
        assertEquals(calibrator.getBiasX(), biasX, 0.0);
        assertEquals(calibrator.getBiasY(), biasY, 0.0);
        assertEquals(calibrator.getBiasZ(), biasZ, 0.0);
    }

    @Test
    public void testGetSetBias() throws LockedException {
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(new double[3], bias1, 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double[] bias2 = bg.getBuffer();

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
    public void testGetSetBiasAsMatrix() throws WrongSizeException, LockedException {
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default value
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(new Matrix(3, 1), bg1);

        // set new value
        final Matrix bg2 = generateBg();

        calibrator.setBias(bg2);

        // check
        final Matrix bg3 = calibrator.getBiasAsMatrix();
        final Matrix bg4 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg4);

        assertEquals(bg2, bg3);
        assertEquals(bg2, bg4);

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
    public void testGetSetBiasAsTriad() throws LockedException {
        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator();

        // check default values
        final AngularSpeedTriad triad1 = calibrator.getBiasAsTriad();
        assertEquals(triad1.getValueX(), 0.0, 0.0);
        assertEquals(triad1.getValueY(), 0.0, 0.0);
        assertEquals(triad1.getValueZ(), 0.0, 0.0);
        assertEquals(triad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

        // set new value
        final Matrix bg = generateBg();
        final AngularSpeedTriad triad2 = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND);
        triad2.setValueCoordinates(bg);

        calibrator.setBias(triad2);

        // check
        final AngularSpeedTriad triad3 = calibrator.getBiasAsTriad();
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        calibrator.getBiasAsTriad(triad4);

        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
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

        final List<FrameBodyKinematics> measurements = new ArrayList<>();
        for (int i = 0; i < KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

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

        final double biasX = bg.getElementAtIndex(0);
        final double biasY = bg.getElementAtIndex(1);
        final double biasZ = bg.getElementAtIndex(2);

        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements,
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

        final Matrix estimatedMg = calibrator.getEstimatedMg();
        final Matrix estimatedGg = calibrator.getEstimatedGg();

        assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
        assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMg, estimatedGg, calibrator);
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

            final double biasX = bg.getElementAtIndex(0);
            final double biasY = bg.getElementAtIndex(1);
            final double biasZ = bg.getElementAtIndex(2);

            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                    new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements,
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

            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultiplePositionsForGeneralCaseWithMinimumMeasuresAndNoNoise()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException {

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
            for (int i = 0; i < KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

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

            final double biasX = bg.getElementAtIndex(0);
            final double biasY = bg.getElementAtIndex(1);
            final double biasZ = bg.getElementAtIndex(2);

            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                    new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements,
                            biasX, biasY, biasZ, false, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            // check
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);

            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
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

        final List<FrameBodyKinematics> measurements = new ArrayList<>();
        for (int i = 0; i < KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

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

        final double biasX = bg.getElementAtIndex(0);
        final double biasY = bg.getElementAtIndex(1);
        final double biasZ = bg.getElementAtIndex(2);

        final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements,
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

        final Matrix estimatedMg = calibrator.getEstimatedMg();
        final Matrix estimatedGg = calibrator.getEstimatedGg();

        assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
        assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

        assertEstimatedResult(estimatedMg, estimatedGg, calibrator);
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

            final double biasX = bg.getElementAtIndex(0);
            final double biasY = bg.getElementAtIndex(1);
            final double biasZ = bg.getElementAtIndex(2);

            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                    new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements,
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

            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultiplePositionsForCommonAxisCaseWithMinimumMeasuresAndNoNoise()
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
            for (int i = 0; i < KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {

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

            final double biasX = bg.getElementAtIndex(0);
            final double biasY = bg.getElementAtIndex(1);
            final double biasZ = bg.getElementAtIndex(2);

            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator =
                    new KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator(measurements,
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

            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(
            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        mCalibrateStart++;
    }

    @Override
    public void onCalibrateEnd(
            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        mCalibrateEnd++;
    }

    private void reset() {
        mCalibrateStart = 0;
        mCalibrateEnd = 0;
    }

    private void checkLocked(
            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator) {
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
            calibrator.setListener(null);
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
            calibrator.setBias((AngularSpeedTriad) null);
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
            final Matrix mg, final Matrix gg,
            final KnownBiasAndFrameGyroscopeLinearLeastSquaresCalibrator calibrator) {

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
