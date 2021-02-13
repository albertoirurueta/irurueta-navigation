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
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.calibration.StandardDeviationFrameBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class KnownFrameGyroscopeNonLinearLeastSquaresCalibratorTest implements
        KnownFrameGyroscopeNonLinearLeastSquaresCalibratorListener {

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

    private static final int LARGE_MEASUREMENT_NUMBER = 100000;

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-5;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;

    private static final int TIMES = 100;

    private int mCalibrateStart;
    private int mCalibrateEnd;

    @Test
    public void testConstructor1() throws WrongSizeException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, biasMatrix2);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeNonLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor2() throws WrongSizeException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, biasMatrix2);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor3() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, biasMatrix2);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor4() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, biasMatrix2);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor5() throws WrongSizeException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(true);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, biasMatrix2);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor6() throws WrongSizeException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, biasMatrix2);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor7() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        true);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, biasMatrix2);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor8() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        true, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = new double[3];
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, biasMatrix2);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(biasMatrix1, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor9() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        initialBiasX, initialBiasY, initialBiasZ);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor10() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        initialBiasX, initialBiasY, initialBiasZ, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor11() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        initialBiasX, initialBiasY, initialBiasZ);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor12() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        initialBiasX, initialBiasY, initialBiasZ, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor13() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, initialBiasX, initialBiasY, initialBiasZ);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor14() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, initialBiasX, initialBiasY, initialBiasZ,
                        this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor15() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        true, initialBiasX, initialBiasY, initialBiasZ);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor16() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        true, initialBiasX, initialBiasY, initialBiasZ,
                        this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor17() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bgx, bgy, bgz);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor18() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bgx, bgy, bgz,
                        this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor19() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        bgx, bgy, bgz);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor20() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        bgx, bgy, bgz, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor21() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, bgx, bgy, bgz);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor22() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, bgx, bgy, bgz, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor23() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        true, bgx, bgy, bgz);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor24() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        true, bgx, bgy, bgz, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor25() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final Matrix mg = generateMg();
        final double initialSx = mg.getElementAt(0, 0);
        final double initialSy = mg.getElementAt(1, 1);
        final double initialSz = mg.getElementAt(2, 2);

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        initialBiasX, initialBiasY, initialBiasZ,
                        initialSx, initialSy, initialSz);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor26() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final Matrix mg = generateMg();
        final double initialSx = mg.getElementAt(0, 0);
        final double initialSy = mg.getElementAt(1, 1);
        final double initialSz = mg.getElementAt(2, 2);

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        initialBiasX, initialBiasY, initialBiasZ,
                        initialSx, initialSy, initialSz);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor27() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final Matrix mg = generateMg();
        final double initialSx = mg.getElementAt(0, 0);
        final double initialSy = mg.getElementAt(1, 1);
        final double initialSz = mg.getElementAt(2, 2);

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        initialBiasX, initialBiasY, initialBiasZ,
                        initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor28() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final Matrix mg = generateMg();
        final double initialSx = mg.getElementAt(0, 0);
        final double initialSy = mg.getElementAt(1, 1);
        final double initialSz = mg.getElementAt(2, 2);

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, initialBiasX, initialBiasY, initialBiasZ,
                        initialSx, initialSy, initialSz);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor29() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final Matrix mg = generateMg();
        final double initialSx = mg.getElementAt(0, 0);
        final double initialSy = mg.getElementAt(1, 1);
        final double initialSz = mg.getElementAt(2, 2);

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, initialBiasX, initialBiasY, initialBiasZ,
                        initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor30() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final Matrix mg = generateMg();
        final double initialSx = mg.getElementAt(0, 0);
        final double initialSy = mg.getElementAt(1, 1);
        final double initialSz = mg.getElementAt(2, 2);

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        true, initialBiasX, initialBiasY, initialBiasZ,
                        initialSx, initialSy, initialSz);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor31() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final Matrix mg = generateMg();
        final double initialSx = mg.getElementAt(0, 0);
        final double initialSy = mg.getElementAt(1, 1);
        final double initialSz = mg.getElementAt(2, 2);

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        true, initialBiasX, initialBiasY, initialBiasZ,
                        initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor32() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final Matrix mg = generateMg();
        final double initialSx = mg.getElementAt(0, 0);
        final double initialSy = mg.getElementAt(1, 1);
        final double initialSz = mg.getElementAt(2, 2);

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        bgx, bgy, bgz, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor33() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final Matrix mg = generateMg();
        final double initialSx = mg.getElementAt(0, 0);
        final double initialSy = mg.getElementAt(1, 1);
        final double initialSz = mg.getElementAt(2, 2);

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        bgx, bgy, bgz, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor34() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final Matrix mg = generateMg();
        final double initialSx = mg.getElementAt(0, 0);
        final double initialSy = mg.getElementAt(1, 1);
        final double initialSz = mg.getElementAt(2, 2);

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        bgx, bgy, bgz, initialSx, initialSy, initialSz);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor35() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final Matrix mg = generateMg();
        final double initialSx = mg.getElementAt(0, 0);
        final double initialSy = mg.getElementAt(1, 1);
        final double initialSz = mg.getElementAt(2, 2);

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        bgx, bgy, bgz, initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor36() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final Matrix mg = generateMg();
        final double initialSx = mg.getElementAt(0, 0);
        final double initialSy = mg.getElementAt(1, 1);
        final double initialSz = mg.getElementAt(2, 2);

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, bgx, bgy, bgz,
                        initialSx, initialSy, initialSz);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor37() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final Matrix mg = generateMg();
        final double initialSx = mg.getElementAt(0, 0);
        final double initialSy = mg.getElementAt(1, 1);
        final double initialSz = mg.getElementAt(2, 2);

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, bgx, bgy, bgz,
                        initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor38() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final Matrix mg = generateMg();
        final double initialSx = mg.getElementAt(0, 0);
        final double initialSy = mg.getElementAt(1, 1);
        final double initialSz = mg.getElementAt(2, 2);

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        true, bgx, bgy, bgz,
                        initialSx, initialSy, initialSz);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor39() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final Matrix mg = generateMg();
        final double initialSx = mg.getElementAt(0, 0);
        final double initialSy = mg.getElementAt(1, 1);
        final double initialSz = mg.getElementAt(2, 2);

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        true, bgx, bgy, bgz,
                        initialSx, initialSy, initialSz, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, 0.0, 0.0,
                        0.0, initialSy, 0.0,
                        0.0, 0.0, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor40() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        initialBiasX, initialBiasY, initialBiasZ,
                        initialSx, initialSy, initialSz, initialMxy, initialMxz,
                        initialMyx, initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor41() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        initialBiasX, initialBiasY, initialBiasZ,
                        initialSx, initialSy, initialSz, initialMxy, initialMxz,
                        initialMyx, initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor42() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        initialBiasX, initialBiasY, initialBiasZ,
                        initialSx, initialSy, initialSz, initialMxy, initialMxz,
                        initialMyx, initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor43() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, initialBiasX, initialBiasY, initialBiasZ,
                        initialSx, initialSy, initialSz, initialMxy, initialMxz,
                        initialMyx, initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor44() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, initialBiasX, initialBiasY, initialBiasZ,
                        initialSx, initialSy, initialSz, initialMxy, initialMxz,
                        initialMyx, initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor45() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        true, initialBiasX, initialBiasY, initialBiasZ,
                        initialSx, initialSy, initialSz, initialMxy, initialMxz,
                        initialMyx, initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor46() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        true, initialBiasX, initialBiasY, initialBiasZ,
                        initialSx, initialSy, initialSz, initialMxy, initialMxz,
                        initialMyx, initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor47() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
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

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        bgx, bgy, bgz, initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor48() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
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

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        bgx, bgy, bgz, initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor49() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
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

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        bgx, bgy, bgz, initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor50() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
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

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        bgx, bgy, bgz, initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor51() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
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

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, bgx, bgy, bgz,
                        initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor52() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
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

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, bgx, bgy, bgz,
                        initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor53() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
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

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        true, bgx, bgy, bgz,
                        initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor54() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
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

        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        true, bgx, bgy, bgz,
                        initialSx, initialSy, initialSz,
                        initialMxy, initialMxz, initialMyx,
                        initialMyz, initialMzx, initialMzy, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);
    }

    @Test
    public void testConstructor55() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final double[] initialBias = bg.getBuffer();
        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        initialBias);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor56() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final double[] initialBias = bg.getBuffer();
        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        initialBias, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor57() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final double[] initialBias = bg.getBuffer();
        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        initialBias);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor58() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final double[] initialBias = bg.getBuffer();
        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        initialBias, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor59() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final double[] initialBias = bg.getBuffer();
        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, initialBias);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor60() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final double[] initialBias = bg.getBuffer();
        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, initialBias, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, new double[1], this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor61() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final double[] initialBias = bg.getBuffer();
        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        true, initialBias);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor62() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        final double[] initialBias = bg.getBuffer();
        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        true, initialBias, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor63() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor64() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                        this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    new Matrix(1, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    new Matrix(3, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor65() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        measurements, bg);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor66() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        measurements, bg, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(1, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(3, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor67() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, bg);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor68() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, bg, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, new Matrix(1, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, new Matrix(3, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor69() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        measurements, true, bg);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true,
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true,
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor70() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        measurements, true, bg, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true,
                    new Matrix(1, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true,
                    new Matrix(3, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor71() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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

        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        bg, mg);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    new Matrix(1, 1), mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    new Matrix(3, 3), mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor72() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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

        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        bg, mg, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    new Matrix(1, 1), mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    new Matrix(3, 3), mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor73() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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

        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        measurements, bg, mg);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(1, 1), mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(3, 3), mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, bg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, bg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor74() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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

        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        measurements, bg, mg, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(1, 1), mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(3, 3), mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, bg, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, bg, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor75() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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

        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, bg, mg);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, new Matrix(1, 1), mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, new Matrix(3, 3), mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, bg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, bg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor76() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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

        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, bg, mg, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, new Matrix(1, 1), mg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, new Matrix(3, 3), mg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, bg, new Matrix(1, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, bg, new Matrix(3, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor77() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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

        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        measurements, true, bg, mg);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, new Matrix(1, 1), mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, new Matrix(3, 3), mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, bg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, bg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor78() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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

        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        measurements, true, bg, mg, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        assertEquals(gg1, gg2);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, new Matrix(1, 1),
                    mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, new Matrix(3, 3),
                    mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, bg, new Matrix(1, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, bg, new Matrix(3, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor79() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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
        final Matrix gg = generateGg();

        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        bg, mg, gg);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    new Matrix(1, 1), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    new Matrix(3, 3), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                    new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                    new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                    mg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                    mg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor80() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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
        final Matrix gg = generateGg();

        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        bg, mg, gg, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    new Matrix(1, 1), mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    new Matrix(3, 3), mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                    new Matrix(1, 3), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                    new Matrix(3, 1), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                    mg, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(bg,
                    mg, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor81() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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
        final Matrix gg = generateGg();

        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        bg, mg, gg);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(1, 1), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(3, 3), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, bg, new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, bg, new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, bg, mg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, bg, mg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor82() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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
        final Matrix gg = generateGg();

        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        bg, mg, gg, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertEquals(calibrator.isCommonAxisUsed(),
                KnownFrameGyroscopeLinearLeastSquaresCalibrator
                        .DEFAULT_USE_COMMON_Z_AXIS);
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(1, 1), mg, gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, new Matrix(3, 3), mg, gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, bg, new Matrix(1, 3), gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, bg, new Matrix(3, 1), gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, bg, mg, new Matrix(1, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, bg, mg, new Matrix(3, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor83() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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
        final Matrix gg = generateGg();

        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, bg, mg, gg);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, new Matrix(1, 1), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, new Matrix(3, 3), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, bg, new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, bg, new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, bg, mg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, bg, mg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor84() throws WrongSizeException {
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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
        final Matrix gg = generateGg();

        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                        true, bg, mg, gg, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertNull(calibrator.getMeasurements());
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, new Matrix(1, 1), mg, gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, new Matrix(3, 3), mg, gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, bg, new Matrix(1, 3), gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, bg, new Matrix(3, 1), gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, bg, mg, new Matrix(1, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    true, bg, mg, new Matrix(3, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor85() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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
        final Matrix gg = generateGg();

        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        true, bg, mg, gg);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertNull(calibrator.getListener());
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, new Matrix(1, 1), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, new Matrix(3, 3), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, bg, new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, bg, new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, bg, mg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, bg, mg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor86() throws WrongSizeException {
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);
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
        final Matrix gg = generateGg();

        KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(measurements,
                        true, bg, mg, gg, this);

        // check default values
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
        AngularSpeed angularSpeed1 = calibrator.getInitialBiasAngularSpeedX();
        AngularSpeed angularSpeed2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedY();
        angularSpeed2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed1 = calibrator.getInitialBiasAngularSpeedZ();
        angularSpeed2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        assertEquals(angularSpeed1, angularSpeed2);
        angularSpeed2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeed2);
        assertEquals(angularSpeed1, angularSpeed2);
        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), initialBiasX, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), initialBiasY, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), initialBiasZ, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
        final double[] bias1 = bg.getBuffer();
        assertArrayEquals(calibrator.getInitialBias(), bias1, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getInitialBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix biasMatrix1 = calibrator.getInitialBiasAsMatrix();
        final Matrix biasMatrix2 = new Matrix(3, 1);
        assertEquals(biasMatrix1, bg);
        calibrator.getInitialBiasAsMatrix(biasMatrix2);
        assertEquals(bg, biasMatrix2);
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        mg2.setSubmatrix(0, 0,
                2, 2,
                new double[]{initialSx, initialMyx, initialMzx,
                        initialMxy, initialSy, initialMzy,
                        initialMxz, initialMyz, initialSz});
        assertEquals(mg1, mg2);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);
        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.STANDARD_DEVIATION_FRAME_BODY_KINEMATICS_MEASUREMENT,
                calibrator.getMeasurementOrSequenceType());
        assertFalse(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertSame(calibrator.getListener(), this);
        assertEquals(KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS,
                calibrator.getMinimumRequiredMeasurementsOrSequences());
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
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
        assertNull(calibrator.getEstimatedBiasAsTriad());
        assertFalse(calibrator.getEstimatedBiasAsTriad(null));
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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
        assertNull(calibrator.getEstimatedBiasXVariance());
        assertNull(calibrator.getEstimatedBiasXStandardDeviation());
        assertNull(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasYVariance());
        assertNull(calibrator.getEstimatedBiasYStandardDeviation());
        assertNull(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasZVariance());
        assertNull(calibrator.getEstimatedBiasZStandardDeviation());
        assertNull(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviation());
        assertFalse(calibrator.getEstimatedBiasStandardDeviation(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverage());
        assertNull(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(null));
        assertNull(calibrator.getEstimatedBiasStandardDeviationNorm());
        assertNull(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed());
        assertFalse(calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(null));
        assertEquals(calibrator.getEstimatedMse(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, new Matrix(1, 1),
                    mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, new Matrix(3, 3),
                    mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, bg,
                    new Matrix(1, 3), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, bg,
                    new Matrix(3, 1), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, bg, mg,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                    measurements, true, bg, mg,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testGetSetInitialBiasX() throws LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialBiasX(), 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);

        calibrator.setInitialBiasX(initialBiasX);

        // check
        assertEquals(initialBiasX, calibrator.getInitialBiasX(), 0.0);
    }

    @Test
    public void testGetSetInitialBiasY() throws LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialBiasY(), 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double initialBiasY = bg.getElementAtIndex(1);

        calibrator.setInitialBiasY(initialBiasY);

        // check
        assertEquals(initialBiasY, calibrator.getInitialBiasY(), 0.0);
    }

    @Test
    public void testGetSetInitialBiasZ() throws LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(0.0, calibrator.getInitialBiasZ(), 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double initialBiasZ = bg.getElementAtIndex(2);

        calibrator.setInitialBiasZ(initialBiasZ);

        // check
        assertEquals(initialBiasZ, calibrator.getInitialBiasZ(), 0.0);
    }

    @Test
    public void testGetSetInitialBiasAngularSpeedX() throws LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        final AngularSpeed biasX1 = calibrator.getInitialBiasAngularSpeedX();

        assertEquals(biasX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(biasX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

        // set new value
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);

        final AngularSpeed biasX2 = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.setInitialBiasX(biasX2);

        // check
        final AngularSpeed biasX3 = calibrator.getInitialBiasAngularSpeedX();
        final AngularSpeed biasX4 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(biasX4);

        assertEquals(biasX2, biasX3);
        assertEquals(biasX2, biasX4);
    }

    @Test
    public void testGetSetInitialBiasAngularSpeedY() throws LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        final AngularSpeed biasY1 = calibrator.getInitialBiasAngularSpeedY();

        assertEquals(biasY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(biasY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

        // set new value
        final Matrix bg = generateBg();
        final double initialBiasY = bg.getElementAtIndex(1);

        final AngularSpeed biasY2 = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.setInitialBiasY(biasY2);

        // check
        final AngularSpeed biasY3 = calibrator.getInitialBiasAngularSpeedY();
        final AngularSpeed biasY4 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(biasY4);

        assertEquals(biasY2, biasY3);
        assertEquals(biasY2, biasY4);
    }

    @Test
    public void testGetSetInitialBiasAngularSpeedZ() throws LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        final AngularSpeed biasZ1 = calibrator.getInitialBiasAngularSpeedZ();

        assertEquals(biasZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(biasZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

        // set new value
        final Matrix bg = generateBg();
        final double initialBiasZ = bg.getElementAtIndex(2);

        final AngularSpeed biasZ2 = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.setInitialBiasZ(biasZ2);

        // check
        final AngularSpeed biasZ3 = calibrator.getInitialBiasAngularSpeedZ();
        final AngularSpeed biasZ4 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(biasZ4);

        assertEquals(biasZ2, biasZ3);
        assertEquals(biasZ2, biasZ4);
    }

    @Test
    public void testSetInitialBiasCoordinates1() throws LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);

        // set new values
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);

        calibrator.setInitialBias(initialBiasX, initialBiasY, initialBiasZ);

        // check
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
    }

    @Test
    public void testSetInitialBiasCoordinates2() throws LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);

        // set new values
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);

        final AngularSpeed bgx = new AngularSpeed(initialBiasX,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy = new AngularSpeed(initialBiasY,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz = new AngularSpeed(initialBiasZ,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setInitialBias(bgx, bgy, bgz);

        // check
        assertEquals(calibrator.getInitialBiasX(), initialBiasX, 0.0);
        assertEquals(calibrator.getInitialBiasY(), initialBiasY, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), initialBiasZ, 0.0);
    }

    @Test
    public void testGetSetInitialBiasAsTriad() throws LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        final AngularSpeedTriad triad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(triad1.getValueX(), 0.0, 0.0);
        assertEquals(triad1.getValueY(), 0.0, 0.0);
        assertEquals(triad1.getValueZ(), 0.0, 0.0);
        assertEquals(triad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad triad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        // set new values
        final Matrix bg = generateBg();
        final double initialBiasX = bg.getElementAtIndex(0);
        final double initialBiasY = bg.getElementAtIndex(1);
        final double initialBiasZ = bg.getElementAtIndex(2);

        final AngularSpeedTriad triad3 = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND,
                initialBiasX, initialBiasY, initialBiasZ);
        calibrator.setInitialBias(triad3);

        final AngularSpeedTriad triad4 = calibrator.getInitialBiasAsTriad();
        final AngularSpeedTriad triad5 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(triad5);

        assertEquals(triad3, triad4);
        assertEquals(triad3, triad5);
    }

    @Test
    public void testGetSetInitialSx() throws WrongSizeException, LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);

        // set new values
        final Matrix mg = generateMg();
        final double initialSx = mg.getElementAt(0, 0);

        calibrator.setInitialSx(initialSx);

        // check
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
    }

    @Test
    public void testGetSetInitialSy() throws WrongSizeException, LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);

        // set new values
        final Matrix mg = generateMg();
        final double initialSy = mg.getElementAt(1, 1);

        calibrator.setInitialSy(initialSy);

        // check
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
    }

    @Test
    public void testGetSetInitialSz() throws WrongSizeException, LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);

        // set new values
        final Matrix mg = generateMg();
        final double initialSz = mg.getElementAt(2, 2);

        calibrator.setInitialSz(initialSz);

        // check
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
    }

    @Test
    public void testGetSetInitialMxy() throws WrongSizeException, LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);

        // set new values
        final Matrix mg = generateMg();
        final double initialMxy = mg.getElementAt(0, 1);

        calibrator.setInitialMxy(initialMxy);

        // check
        assertEquals(calibrator.getInitialMxy(), initialMxy, 0.0);
    }

    @Test
    public void testGetSetInitialMxz() throws WrongSizeException, LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);

        // set new values
        final Matrix mg = generateMg();
        final double initialMxz = mg.getElementAt(0, 2);

        calibrator.setInitialMxz(initialMxz);

        // check
        assertEquals(calibrator.getInitialMxz(), initialMxz, 0.0);
    }

    @Test
    public void testGetSetInitialMyx() throws WrongSizeException, LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);

        // set new values
        final Matrix mg = generateMg();
        final double initialMyx = mg.getElementAt(1, 0);

        calibrator.setInitialMyx(initialMyx);

        // check
        assertEquals(calibrator.getInitialMyx(), initialMyx, 0.0);
    }

    @Test
    public void testGetSetInitialMyz() throws WrongSizeException, LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);

        // set new values
        final Matrix mg = generateMg();
        final double initialMyz = mg.getElementAt(1, 2);

        calibrator.setInitialMyz(initialMyz);

        // check
        assertEquals(calibrator.getInitialMyz(), initialMyz, 0.0);
    }

    @Test
    public void testGetSetInitialMzx() throws WrongSizeException, LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);

        // set new values
        final Matrix mg = generateMg();
        final double initialMzx = mg.getElementAt(2, 0);

        calibrator.setInitialMzx(initialMzx);

        // check
        assertEquals(calibrator.getInitialMzx(), initialMzx, 0.0);
    }

    @Test
    public void testGetSetInitialMzy() throws WrongSizeException, LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        // set new values
        final Matrix mg = generateMg();
        final double initialMzy = mg.getElementAt(2, 1);

        calibrator.setInitialMzy(initialMzy);

        // check
        assertEquals(calibrator.getInitialMzy(), initialMzy, 0.0);
    }

    @Test
    public void testSetInitialScalingFactors() throws WrongSizeException,
            LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);

        // set new values
        final Matrix mg = generateMg();
        final double initialSx = mg.getElementAt(0, 0);
        final double initialSy = mg.getElementAt(1, 1);
        final double initialSz = mg.getElementAt(2, 2);

        calibrator.setInitialScalingFactors(initialSx, initialSy, initialSz);

        // check
        assertEquals(calibrator.getInitialSx(), initialSx, 0.0);
        assertEquals(calibrator.getInitialSy(), initialSy, 0.0);
        assertEquals(calibrator.getInitialSz(), initialSz, 0.0);
    }

    @Test
    public void testSetInitialCrossCouplingErrors() throws WrongSizeException,
            LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

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
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

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
                initialSx, initialSy, initialSz,
                initialMxy, initialMxz, initialMyx,
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
    public void testGetSetInitialBiasArray() throws LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertArrayEquals(calibrator.getInitialBias(), new double[3], 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double[] initialBias1 = bg.getBuffer();

        calibrator.setInitialBias(initialBias1);

        // check
        final double[] initialBias2 = calibrator.getInitialBias();
        final double[] initialBias3 = new double[3];
        calibrator.getInitialBias(initialBias3);

        assertArrayEquals(initialBias1, initialBias2, 0.0);
        assertArrayEquals(initialBias1, initialBias3, 0.0);

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
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertEquals(calibrator.getInitialBiasAsMatrix(),
                new Matrix(3, 1));

        // set new value
        final Matrix bg = generateBg();
        calibrator.setInitialBias(bg);

        // check
        final Matrix initialBias1 = calibrator.getInitialBiasAsMatrix();
        final Matrix initialBias2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(initialBias2);

        assertEquals(bg, initialBias1);
        assertEquals(bg, initialBias2);

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
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default values
        final Matrix initialMg1 = calibrator.getInitialMg();

        assertEquals(initialMg1, new Matrix(3, 3));

        // set new value
        final Matrix mg = generateMg();

        calibrator.setInitialMg(mg);

        // check
        final Matrix initialMg2 = calibrator.getInitialMg();
        final Matrix initialMg3 = new Matrix(3, 3);
        calibrator.getInitialMg(initialMg3);

        assertEquals(mg, initialMg2);
        assertEquals(mg, initialMg3);

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
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check
        assertEquals(calibrator.getInitialGg(), new Matrix(3, 3));

        // set new value
        final Matrix gg = generateGg();

        calibrator.setInitialGg(gg);

        // check
        final Matrix initialGg1 = calibrator.getInitialGg();
        final Matrix initialGg2 = new Matrix(3, 3);
        calibrator.getInitialGg(initialGg2);

        assertEquals(gg, initialGg1);
        assertEquals(gg, initialGg2);

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
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getMeasurements());

        // set new value
        final Collection<StandardDeviationFrameBodyKinematics> measurements =
                Collections.emptyList();
        calibrator.setMeasurements(measurements);

        // check
        assertSame(calibrator.getMeasurements(), measurements);
    }

    @Test
    public void testIsSetCommonAxisUsed() throws LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertFalse(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(true);

        // check
        assertTrue(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testIsReady() throws LockedException {
        final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator();

        // check default value
        assertFalse(calibrator.isReady());

        // set enough measurements
        final List<StandardDeviationFrameBodyKinematics> measurements1 =
                Arrays.asList(new StandardDeviationFrameBodyKinematics(),
                        new StandardDeviationFrameBodyKinematics(),
                        new StandardDeviationFrameBodyKinematics(),
                        new StandardDeviationFrameBodyKinematics(),
                        new StandardDeviationFrameBodyKinematics(),
                        new StandardDeviationFrameBodyKinematics(),
                        new StandardDeviationFrameBodyKinematics());

        calibrator.setMeasurements(measurements1);

        assertTrue(calibrator.isReady());

        // set too few measurements
        final List<StandardDeviationFrameBodyKinematics> measurements2 =
                Arrays.asList(new StandardDeviationFrameBodyKinematics(),
                        new StandardDeviationFrameBodyKinematics(),
                        new StandardDeviationFrameBodyKinematics(),
                        new StandardDeviationFrameBodyKinematics(),
                        new StandardDeviationFrameBodyKinematics(),
                        new StandardDeviationFrameBodyKinematics());

        calibrator.setMeasurements(measurements2);

        assertFalse(calibrator.isReady());
    }

    @Test
    public void testCalibrateMultipleOrientationsForGeneralCasewithMinimumMeasuresAndNoNoise()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException {

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
            final double latitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final List<StandardDeviationFrameBodyKinematics> measurements =
                    new ArrayList<>();
            for (int i = 0; i < KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {
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
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                    new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                            measurements, false, bg, mg, gg, this);

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

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            assertTrue(bg.equals(estimatedBg, ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedChiSq() < 0.0);
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultipleOrientationsForGeneralCaseWithNoiseLargeNumberOfMeasurements()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, CalibrationException {

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

            final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                    gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final List<StandardDeviationFrameBodyKinematics> measurements =
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
                final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                        .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random);

                final StandardDeviationFrameBodyKinematics measurement =
                        new StandardDeviationFrameBodyKinematics(measuredKinematics,
                                ecefFrame, ecefFrame, TIME_INTERVAL_SECONDS,
                                specificForceStandardDeviation,
                                angularRateStandardDeviation);
                measurements.add(measurement);
            }

            final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                    new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                            measurements, false, this);

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
            checkGeneralCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultiplePositionsForGeneralCaseWithMinimumMeasuresAndNoNoise()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException {

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

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final List<StandardDeviationFrameBodyKinematics> measurements =
                    new ArrayList<>();
            for (int i = 0; i < KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {
                final double latitude = Math.toRadians(
                        randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
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
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                    new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                            measurements, false, bg, mg, gg, this);

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

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertNotNull(estimatedMg);
            assertNotNull(estimatedGg);

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedChiSq() < 0.0);
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultipleOrientationsForCommonAxisCaseWithMinimumMeasuresAndNoNoise()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException {

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
            final double latitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final List<StandardDeviationFrameBodyKinematics> measurements =
                    new ArrayList<>();
            for (int i = 0; i < KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {
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
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                    new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                            measurements, true, bg, mg, gg, this);

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

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            assertTrue(bg.equals(estimatedBg, ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedChiSq() < 0.0);
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultipleOrientationsForCommonAxisCaseWithNoiseLargeNumberOfMeasurements()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, CalibrationException {

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

            final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                    gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final List<StandardDeviationFrameBodyKinematics> measurements =
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
                final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                        .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random);

                final StandardDeviationFrameBodyKinematics measurement =
                        new StandardDeviationFrameBodyKinematics(measuredKinematics,
                                ecefFrame, ecefFrame, TIME_INTERVAL_SECONDS,
                                specificForceStandardDeviation,
                                angularRateStandardDeviation);
                measurements.add(measurement);
            }

            final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                    new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
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
            checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateMultiplePositionsForCommonAxisCaseWithMinimumMeasuresAndNoNoise()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException {

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

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final List<StandardDeviationFrameBodyKinematics> measurements =
                    new ArrayList<>();
            for (int i = 0; i < KnownFrameGyroscopeNonLinearLeastSquaresCalibrator.MINIMUM_MEASUREMENTS; i++) {
                final double latitude = Math.toRadians(
                        randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
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
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator =
                    new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator(
                            measurements, true, bg, mg, gg, this);

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

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertNotNull(estimatedMg);
            assertNotNull(estimatedGg);

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedChiSq() < 0.0);
            assertTrue(calibrator.getEstimatedMse() > 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        mCalibrateStart++;
    }

    @Override
    public void onCalibrateEnd(
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator) {
        checkLocked(calibrator);
        mCalibrateEnd++;
    }

    private void reset() {
        mCalibrateStart = 0;
        mCalibrateEnd = 0;
    }

    private void checkLocked(
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator) {
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
            calibrator.setInitialBias((AngularSpeedTriad) null);
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
            calibrator.calibrate();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final Exception e) {
            fail("LockedException expected but not thrown");
        }
    }

    private void assertEstimatedResult(
            final Matrix bg, final Matrix mg, final Matrix gg,
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator)
            throws WrongSizeException {

        final double[] estimatedBiases = calibrator.getEstimatedBiases();
        assertArrayEquals(bg.getBuffer(), estimatedBiases, 0.0);

        final double[] estimatedBiases2 = new double[3];
        calibrator.getEstimatedBiases(estimatedBiases2);
        assertArrayEquals(estimatedBiases, estimatedBiases2, 0.0);

        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getEstimatedBiasesAsMatrix(bg2);

        assertEquals(bg, bg2);

        assertEquals(bg.getElementAtIndex(0), calibrator.getEstimatedBiasX(),
                0.0);
        assertEquals(bg.getElementAtIndex(1), calibrator.getEstimatedBiasY(),
                0.0);
        assertEquals(bg.getElementAtIndex(2), calibrator.getEstimatedBiasZ(),
                0.0);

        final AngularSpeed bgx1 = calibrator.getEstimatedBiasAngularSpeedX();
        final AngularSpeed bgx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getEstimatedBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);
        assertEquals(calibrator.getEstimatedBiasX(),
                bgx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());

        final AngularSpeed bgy1 = calibrator.getEstimatedBiasAngularSpeedY();
        final AngularSpeed bgy2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getEstimatedBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);
        assertEquals(calibrator.getEstimatedBiasY(),
                bgy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());

        final AngularSpeed bgz1 = calibrator.getEstimatedBiasAngularSpeedZ();
        final AngularSpeed bgz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getEstimatedBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);
        assertEquals(calibrator.getEstimatedBiasZ(),
                bgz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());

        final AngularSpeedTriad bTriad1 = calibrator.getEstimatedBiasAsTriad();
        assertEquals(calibrator.getEstimatedBiasX(), bTriad1.getValueX(), 0.0);
        assertEquals(calibrator.getEstimatedBiasY(), bTriad1.getValueY(), 0.0);
        assertEquals(calibrator.getEstimatedBiasZ(), bTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bTriad1.getUnit());
        final AngularSpeedTriad bTriad2 = new AngularSpeedTriad();
        calibrator.getEstimatedBiasAsTriad(bTriad2);
        assertEquals(bTriad1, bTriad2);

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

        assertCovariance(calibrator);
    }

    private void assertCovariance(
            final KnownFrameGyroscopeNonLinearLeastSquaresCalibrator calibrator) {
        assertNotNull(calibrator.getEstimatedBiasXVariance());

        assertNotNull(calibrator.getEstimatedBiasXVariance());
        assertNotNull(calibrator.getEstimatedBiasXStandardDeviation());
        final AngularSpeed stdBx1 = calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed();
        assertNotNull(stdBx1);
        final AngularSpeed stdBx2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertTrue(calibrator.getEstimatedBiasXStandardDeviationAsAngularSpeed(stdBx2));
        assertEquals(stdBx1, stdBx2);

        assertNotNull(calibrator.getEstimatedBiasYVariance());
        assertNotNull(calibrator.getEstimatedBiasYStandardDeviation());
        final AngularSpeed stdBy1 = calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed();
        assertNotNull(stdBy1);
        final AngularSpeed stdBy2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertTrue(calibrator.getEstimatedBiasYStandardDeviationAsAngularSpeed(stdBy2));
        assertEquals(stdBy1, stdBy2);

        assertNotNull(calibrator.getEstimatedBiasZVariance());
        assertNotNull(calibrator.getEstimatedBiasZStandardDeviation());
        final AngularSpeed stdBz1 = calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed();
        assertNotNull(stdBz1);
        final AngularSpeed stdBz2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        assertTrue(calibrator.getEstimatedBiasZStandardDeviationAsAngularSpeed(stdBz2));
        assertEquals(stdBz1, stdBz2);

        final AngularSpeedTriad std1 = calibrator.getEstimatedBiasStandardDeviation();
        assertEquals(calibrator.getEstimatedBiasXStandardDeviation(), std1.getValueX(), 0.0);
        assertEquals(calibrator.getEstimatedBiasYStandardDeviation(), std1.getValueY(), 0.0);
        assertEquals(calibrator.getEstimatedBiasZStandardDeviation(), std1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, std1.getUnit());
        final AngularSpeedTriad std2 = new AngularSpeedTriad();
        calibrator.getEstimatedBiasStandardDeviation(std2);

        final double avgStd = (calibrator.getEstimatedBiasXStandardDeviation() +
                calibrator.getEstimatedBiasYStandardDeviation() +
                calibrator.getEstimatedBiasZStandardDeviation()) / 3.0;
        assertEquals(avgStd, calibrator.getEstimatedBiasStandardDeviationAverage(), 0.0);
        final AngularSpeed avg1 = calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed();
        assertEquals(avgStd, avg1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, avg1.getUnit());
        final AngularSpeed avg2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getEstimatedBiasStandardDeviationAverageAsAngularSpeed(avg2);
        assertEquals(avg1, avg2);

        assertEquals(std1.getNorm(), calibrator.getEstimatedBiasStandardDeviationNorm(), ABSOLUTE_ERROR);
        final AngularSpeed norm1 = calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed();
        assertEquals(std1.getNorm(), norm1.getValue().doubleValue(), ABSOLUTE_ERROR);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, norm1.getUnit());
        final AngularSpeed norm2 = new AngularSpeed(1.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getEstimatedBiasStandardDeviationNormAsAngularSpeed(norm2);
        assertEquals(norm1, norm2);
    }

    private void checkCommonAxisCovariance(final Matrix covariance) {
        assertEquals(covariance.getRows(), 21);
        assertEquals(covariance.getColumns(), 21);

        for (int j = 0; j < 18; j++) {
            final boolean colIsZero = j == 8 || j == 10 || j == 11;
            for (int i = 0; i < 18; i++) {
                final boolean rowIsZero = i == 8 || i == 10 || i == 11;
                if (colIsZero || rowIsZero) {
                    assertEquals(covariance.getElementAt(i, j), 0.0, 0.0);
                }
            }
        }
    }

    private void checkGeneralCovariance(final Matrix covariance) {
        assertEquals(covariance.getRows(), 21);
        assertEquals(covariance.getColumns(), 21);

        for (int i = 0; i < 21; i++) {
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
