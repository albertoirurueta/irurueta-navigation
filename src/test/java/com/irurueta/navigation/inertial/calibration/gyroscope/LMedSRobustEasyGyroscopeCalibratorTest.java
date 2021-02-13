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
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
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
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;
import static org.junit.Assert.assertEquals;

public class LMedSRobustEasyGyroscopeCalibratorTest implements
        RobustEasyGyroscopeCalibratorListener {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_ANGLE_VARIATION_DEGREES = -2.0;
    private static final double MAX_ANGLE_VARIATION_DEGREES = 2.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;
    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;
    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final int MEASUREMENT_NUMBER = 1000;

    private static final int OUTLIER_PERCENTAGE = 5;

    private static final double THRESHOLD = 1e-3;

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-5;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;

    private static final double OUTLIER_ERROR_FACTOR = 1000.0;

    private static final int TIMES = 100;

    private int mCalibrateStart;
    private int mCalibrateEnd;
    private int mCalibrateNextIteration;
    private int mCalibrateProgressChange;

    @Test
    public void testConstructor1() throws WrongSizeException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getAccelerometerBiasX(),
                0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(),
                0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(),
                0.0, 0.0);
        final Acceleration accelerationX1 = calibrator
                .getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator
                .getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator
                .getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, new double[3], 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(calibrator.getAccelerometerSx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), 0.0, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);

        final AngularSpeed angularSpeedX1 = calibrator
                .getInitialBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), 0.0,
                0.0);
        assertEquals(angularSpeedX1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator
                .getInitialBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), 0.0,
                0.0);
        assertEquals(angularSpeedY1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator
                .getInitialBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), 0.0,
                0.0);
        assertEquals(angularSpeedZ1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

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

        final double[] bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, new double[3], 0.0);
        final double[] bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, new Matrix(3, 1));
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

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

        assertNull(calibrator.getSequences());

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 19);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(calibrator.getProgressDelta(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getPreliminarySubsetSize(),
                EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);
    }

    @Test
    public void testConstructor2() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator(
                        sequences, bg, mg, gg);

        // check default values
        assertEquals(calibrator.getAccelerometerBiasX(),
                0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(),
                0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(),
                0.0, 0.0);
        final Acceleration accelerationX1 = calibrator
                .getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator
                .getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator
                .getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, new double[3], 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(calibrator.getAccelerometerSx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), 0.0, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getInitialBiasX(), bgx, 0.0);
        assertEquals(calibrator.getInitialBiasY(), bgy, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), bgz, 0.0);

        final AngularSpeed angularSpeedX1 = calibrator
                .getInitialBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx,
                0.0);
        assertEquals(angularSpeedX1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator
                .getInitialBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy,
                0.0);
        assertEquals(angularSpeedY1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator
                .getInitialBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz,
                0.0);
        assertEquals(angularSpeedZ1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), bgx, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), bgy, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);

        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bg.getBuffer(), 0.0);
        final double[] bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);

        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);

        assertSame(calibrator.getSequences(), sequences);

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 19);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(calibrator.getProgressDelta(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getPreliminarySubsetSize(),
                EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, new Matrix(1, 1), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, new Matrix(3, 3), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, mg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, mg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor3() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator(
                        sequences, bg, mg, gg, this);

        // check default values
        assertEquals(calibrator.getAccelerometerBiasX(),
                0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(),
                0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(),
                0.0, 0.0);
        final Acceleration accelerationX1 = calibrator
                .getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator
                .getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator
                .getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, new double[3], 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(calibrator.getAccelerometerSx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), 0.0, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getInitialBiasX(), bgx, 0.0);
        assertEquals(calibrator.getInitialBiasY(), bgy, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), bgz, 0.0);

        final AngularSpeed angularSpeedX1 = calibrator
                .getInitialBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx,
                0.0);
        assertEquals(angularSpeedX1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator
                .getInitialBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy,
                0.0);
        assertEquals(angularSpeedY1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator
                .getInitialBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz,
                0.0);
        assertEquals(angularSpeedZ1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), bgx, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), bgy, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);

        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bg.getBuffer(), 0.0);
        final double[] bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);

        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);

        assertSame(calibrator.getSequences(), sequences);

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 19);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(calibrator.getProgressDelta(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getPreliminarySubsetSize(),
                EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, new Matrix(1, 1), mg, gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, new Matrix(3, 3), mg, gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, new Matrix(1, 3), gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, new Matrix(3, 1), gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, mg, new Matrix(1, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, mg, new Matrix(3, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor4() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator(
                        sequences, bias, mg, gg);

        // check default values
        assertEquals(calibrator.getAccelerometerBiasX(),
                0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(),
                0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(),
                0.0, 0.0);
        final Acceleration accelerationX1 = calibrator
                .getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator
                .getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator
                .getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, new double[3], 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(calibrator.getAccelerometerSx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), 0.0, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getInitialBiasX(), bgx, 0.0);
        assertEquals(calibrator.getInitialBiasY(), bgy, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), bgz, 0.0);

        final AngularSpeed angularSpeedX1 = calibrator
                .getInitialBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx,
                0.0);
        assertEquals(angularSpeedX1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator
                .getInitialBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy,
                0.0);
        assertEquals(angularSpeedY1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator
                .getInitialBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz,
                0.0);
        assertEquals(angularSpeedZ1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), bgx, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), bgy, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);

        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);

        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);

        assertSame(calibrator.getSequences(), sequences);

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 19);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(calibrator.getProgressDelta(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getPreliminarySubsetSize(),
                EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, new double[1], mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, mg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, mg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor5() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator(
                        sequences, bias, mg, gg, this);

        // check default values
        assertEquals(calibrator.getAccelerometerBiasX(),
                0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(),
                0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(),
                0.0, 0.0);
        final Acceleration accelerationX1 = calibrator
                .getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator
                .getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator
                .getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, new double[3], 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(calibrator.getAccelerometerSx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), 0.0, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getInitialBiasX(), bgx, 0.0);
        assertEquals(calibrator.getInitialBiasY(), bgy, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), bgz, 0.0);

        final AngularSpeed angularSpeedX1 = calibrator
                .getInitialBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx,
                0.0);
        assertEquals(angularSpeedX1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator
                .getInitialBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy,
                0.0);
        assertEquals(angularSpeedY1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator
                .getInitialBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz,
                0.0);
        assertEquals(angularSpeedZ1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), bgx, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), bgy, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);

        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);

        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);

        assertSame(calibrator.getSequences(), sequences);

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 19);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(calibrator.getProgressDelta(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getPreliminarySubsetSize(),
                EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, new double[1], mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, new Matrix(1, 3), gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, new Matrix(3, 1), gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, mg, new Matrix(1, 3),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, mg, new Matrix(3, 1),
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor6() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator(
                        sequences, bias, mg, gg, baArray, ma);

        // check default values
        assertEquals(calibrator.getAccelerometerBiasX(),
                baX, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(),
                baY, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(),
                baZ, 0.0);
        final Acceleration accelerationX1 = calibrator
                .getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(),
                baX, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator
                .getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(),
                baY, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator
                .getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(),
                baZ, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(calibrator.getAccelerometerSx(), sxa, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), sya, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), sza, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), mxya, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), mxza, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), myxa, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), myza, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), mzxa, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), mzya, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getInitialBiasX(), bgx, 0.0);
        assertEquals(calibrator.getInitialBiasY(), bgy, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), bgz, 0.0);

        final AngularSpeed angularSpeedX1 = calibrator
                .getInitialBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx,
                0.0);
        assertEquals(angularSpeedX1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator
                .getInitialBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy,
                0.0);
        assertEquals(angularSpeedY1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator
                .getInitialBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz,
                0.0);
        assertEquals(angularSpeedZ1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), bgx, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), bgy, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);

        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);

        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);

        assertSame(calibrator.getSequences(), sequences);

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 19);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(calibrator.getProgressDelta(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getPreliminarySubsetSize(),
                EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, new double[1], mg, gg, baArray, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, new Matrix(1, 3), gg,
                    baArray, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, new Matrix(3, 1), gg,
                    baArray, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, mg, new Matrix(1, 3),
                    baArray, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, mg, new Matrix(3, 1),
                    baArray, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, mg, gg, new double[1], ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, mg, gg, baArray,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, mg, gg, baArray,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor7() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator(
                        sequences, bias, mg, gg, baArray, ma, this);

        // check default values
        assertEquals(calibrator.getAccelerometerBiasX(),
                baX, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(),
                baY, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(),
                baZ, 0.0);
        final Acceleration accelerationX1 = calibrator
                .getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(),
                baX, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator
                .getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(),
                baY, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator
                .getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(),
                baZ, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(calibrator.getAccelerometerSx(), sxa, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), sya, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), sza, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), mxya, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), mxza, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), myxa, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), myza, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), mzxa, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), mzya, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getInitialBiasX(), bgx, 0.0);
        assertEquals(calibrator.getInitialBiasY(), bgy, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), bgz, 0.0);

        final AngularSpeed angularSpeedX1 = calibrator
                .getInitialBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx,
                0.0);
        assertEquals(angularSpeedX1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator
                .getInitialBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy,
                0.0);
        assertEquals(angularSpeedY1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator
                .getInitialBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz,
                0.0);
        assertEquals(angularSpeedZ1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), bgx, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), bgy, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);

        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);

        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);

        assertSame(calibrator.getSequences(), sequences);

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 19);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(calibrator.getProgressDelta(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getPreliminarySubsetSize(),
                EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, new double[1], mg, gg, baArray, ma,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, new Matrix(1, 3), gg,
                    baArray, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, new Matrix(3, 1), gg,
                    baArray, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, mg, new Matrix(1, 3),
                    baArray, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, mg, new Matrix(3, 1),
                    baArray, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, mg, gg, new double[1], ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, mg, gg, baArray,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bias, mg, gg, baArray,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor8() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator(
                        sequences, bg, mg, gg, ba, ma);

        // check default values
        assertEquals(calibrator.getAccelerometerBiasX(),
                baX, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(),
                baY, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(),
                baZ, 0.0);
        final Acceleration accelerationX1 = calibrator
                .getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(),
                baX, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator
                .getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(),
                baY, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator
                .getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(),
                baZ, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(calibrator.getAccelerometerSx(), sxa, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), sya, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), sza, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), mxya, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), mxza, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), myxa, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), myza, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), mzxa, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), mzya, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getInitialBiasX(), bgx, 0.0);
        assertEquals(calibrator.getInitialBiasY(), bgy, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), bgz, 0.0);

        final AngularSpeed angularSpeedX1 = calibrator
                .getInitialBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx,
                0.0);
        assertEquals(angularSpeedX1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator
                .getInitialBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy,
                0.0);
        assertEquals(angularSpeedY1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator
                .getInitialBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz,
                0.0);
        assertEquals(angularSpeedZ1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), bgx, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), bgy, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);

        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);

        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);

        assertSame(calibrator.getSequences(), sequences);

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 19);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(calibrator.getProgressDelta(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getPreliminarySubsetSize(),
                EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, new Matrix(1, 1), mg,
                    gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, new Matrix(3, 3), mg,
                    gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, new Matrix(1, 3),
                    gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, new Matrix(3, 1),
                    gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, mg, new Matrix(1, 3),
                    ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, mg, new Matrix(3, 1),
                    ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, mg, gg, new Matrix(1, 1),
                    ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, mg, gg, new Matrix(3, 3),
                    ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, mg, gg, ba,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, mg, gg, ba,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor9() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator(
                        sequences, bg, mg, gg, ba, ma, this);

        // check default values
        assertEquals(calibrator.getAccelerometerBiasX(),
                baX, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(),
                baY, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(),
                baZ, 0.0);
        final Acceleration accelerationX1 = calibrator
                .getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(),
                baX, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator
                .getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(),
                baY, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator
                .getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(),
                baZ, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(calibrator.getAccelerometerSx(), sxa, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), sya, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), sza, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), mxya, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), mxza, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), myxa, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), myza, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), mzxa, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), mzya, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getInitialBiasX(), bgx, 0.0);
        assertEquals(calibrator.getInitialBiasY(), bgy, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), bgz, 0.0);

        final AngularSpeed angularSpeedX1 = calibrator
                .getInitialBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx,
                0.0);
        assertEquals(angularSpeedX1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator
                .getInitialBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy,
                0.0);
        assertEquals(angularSpeedY1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator
                .getInitialBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz,
                0.0);
        assertEquals(angularSpeedZ1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), bgx, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), bgy, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);

        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);

        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);

        assertSame(calibrator.getSequences(), sequences);

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 19);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(calibrator.getProgressDelta(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getPreliminarySubsetSize(),
                EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, new Matrix(1, 1), mg,
                    gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, new Matrix(3, 3), mg,
                    gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, new Matrix(1, 3),
                    gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, new Matrix(3, 1),
                    gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, mg, new Matrix(1, 3),
                    ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, mg, new Matrix(3, 1),
                    ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, mg, gg, new Matrix(1, 1),
                    ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, mg, gg, new Matrix(3, 3),
                    ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, mg, gg, ba,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, bg, mg, gg, ba,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor10() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator(
                        sequences, false,
                        false, bg, mg, gg);

        // check default values
        assertEquals(calibrator.getAccelerometerBiasX(),
                0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(),
                0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(),
                0.0, 0.0);
        final Acceleration accelerationX1 = calibrator
                .getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator
                .getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator
                .getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, new double[3], 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(calibrator.getAccelerometerSx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), 0.0, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getInitialBiasX(), bgx, 0.0);
        assertEquals(calibrator.getInitialBiasY(), bgy, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), bgz, 0.0);

        final AngularSpeed angularSpeedX1 = calibrator
                .getInitialBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx,
                0.0);
        assertEquals(angularSpeedX1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator
                .getInitialBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy,
                0.0);
        assertEquals(angularSpeedY1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator
                .getInitialBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz,
                0.0);
        assertEquals(angularSpeedZ1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), bgx, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), bgy, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);

        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bg.getBuffer(), 0.0);
        final double[] bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);

        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);

        assertSame(calibrator.getSequences(), sequences);

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(calibrator.getProgressDelta(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getPreliminarySubsetSize(),
                EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false,
                    new Matrix(1, 1), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false,
                    new Matrix(3, 3), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg,
                    new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg,
                    new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg,
                    mg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg,
                    mg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor11() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator(
                        sequences, false,
                        false, bg, mg, gg,
                        this);

        // check default values
        assertEquals(calibrator.getAccelerometerBiasX(),
                0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(),
                0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(),
                0.0, 0.0);
        final Acceleration accelerationX1 = calibrator
                .getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator
                .getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator
                .getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, new double[3], 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(calibrator.getAccelerometerSx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), 0.0, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getInitialBiasX(), bgx, 0.0);
        assertEquals(calibrator.getInitialBiasY(), bgy, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), bgz, 0.0);

        final AngularSpeed angularSpeedX1 = calibrator
                .getInitialBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx,
                0.0);
        assertEquals(angularSpeedX1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator
                .getInitialBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy,
                0.0);
        assertEquals(angularSpeedY1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator
                .getInitialBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz,
                0.0);
        assertEquals(angularSpeedZ1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), bgx, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), bgy, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);

        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bg.getBuffer(), 0.0);
        final double[] bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);

        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);

        assertSame(calibrator.getSequences(), sequences);

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(calibrator.getProgressDelta(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getPreliminarySubsetSize(),
                EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false,
                    new Matrix(1, 1), mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false,
                    new Matrix(3, 3), mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg,
                    new Matrix(1, 3), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg,
                    new Matrix(3, 1), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg,
                    mg, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg,
                    mg, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor12() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator(
                        sequences, false,
                        false, bias, mg, gg);

        // check default values
        assertEquals(calibrator.getAccelerometerBiasX(),
                0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(),
                0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(),
                0.0, 0.0);
        final Acceleration accelerationX1 = calibrator
                .getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator
                .getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator
                .getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, new double[3], 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(calibrator.getAccelerometerSx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), 0.0, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getInitialBiasX(), bgx, 0.0);
        assertEquals(calibrator.getInitialBiasY(), bgy, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), bgz, 0.0);

        final AngularSpeed angularSpeedX1 = calibrator
                .getInitialBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx,
                0.0);
        assertEquals(angularSpeedX1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator
                .getInitialBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy,
                0.0);
        assertEquals(angularSpeedY1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator
                .getInitialBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz,
                0.0);
        assertEquals(angularSpeedZ1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), bgx, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), bgy, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);

        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);

        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);

        assertSame(calibrator.getSequences(), sequences);

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(calibrator.getProgressDelta(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getPreliminarySubsetSize(),
                EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false,
                    new double[1], mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias,
                    new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias,
                    new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias, mg,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias, mg,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor13() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator(
                        sequences, false,
                        false, bias, mg, gg,
                        this);

        // check default values
        assertEquals(calibrator.getAccelerometerBiasX(),
                0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(),
                0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(),
                0.0, 0.0);
        final Acceleration accelerationX1 = calibrator
                .getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator
                .getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator
                .getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(),
                0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, new double[3], 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, new Matrix(3, 1));
        final Matrix ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(calibrator.getAccelerometerSx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), 0.0, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, new Matrix(3, 3));
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getInitialBiasX(), bgx, 0.0);
        assertEquals(calibrator.getInitialBiasY(), bgy, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), bgz, 0.0);

        final AngularSpeed angularSpeedX1 = calibrator
                .getInitialBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx,
                0.0);
        assertEquals(angularSpeedX1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator
                .getInitialBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy,
                0.0);
        assertEquals(angularSpeedY1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator
                .getInitialBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz,
                0.0);
        assertEquals(angularSpeedZ1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), bgx, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), bgy, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);

        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);

        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);

        assertSame(calibrator.getSequences(), sequences);

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(calibrator.getProgressDelta(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getPreliminarySubsetSize(),
                EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false,
                    new double[1], mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias,
                    new Matrix(1, 3), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias,
                    new Matrix(3, 1), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias, mg,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias, mg,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor14() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator(
                        sequences, false,
                        false, bias, mg, gg, baArray, ma);

        // check default values
        assertEquals(calibrator.getAccelerometerBiasX(),
                baX, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(),
                baY, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(),
                baZ, 0.0);
        final Acceleration accelerationX1 = calibrator
                .getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(),
                baX, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator
                .getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(),
                baY, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator
                .getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(),
                baZ, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(calibrator.getAccelerometerSx(), sxa, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), sya, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), sza, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), mxya, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), mxza, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), myxa, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), myza, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), mzxa, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), mzya, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getInitialBiasX(), bgx, 0.0);
        assertEquals(calibrator.getInitialBiasY(), bgy, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), bgz, 0.0);

        final AngularSpeed angularSpeedX1 = calibrator
                .getInitialBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx,
                0.0);
        assertEquals(angularSpeedX1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator
                .getInitialBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy,
                0.0);
        assertEquals(angularSpeedY1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator
                .getInitialBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz,
                0.0);
        assertEquals(angularSpeedZ1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), bgx, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), bgy, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);

        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);

        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);

        assertSame(calibrator.getSequences(), sequences);

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(calibrator.getProgressDelta(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getPreliminarySubsetSize(),
                EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false,
                    new double[1], mg, gg, baArray, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias,
                    new Matrix(1, 3), gg,
                    baArray, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias,
                    new Matrix(3, 1), gg,
                    baArray, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias,
                    mg, new Matrix(1, 3),
                    baArray, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias,
                    mg, new Matrix(3, 1),
                    baArray, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias,
                    mg, gg, new double[1], ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias,
                    mg, gg, baArray,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias,
                    mg, gg, baArray,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor15() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator(
                        sequences, false,
                        false,
                        bias, mg, gg, baArray, ma, this);

        // check default values
        assertEquals(calibrator.getAccelerometerBiasX(),
                baX, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(),
                baY, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(),
                baZ, 0.0);
        final Acceleration accelerationX1 = calibrator
                .getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(),
                baX, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator
                .getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(),
                baY, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator
                .getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(),
                baZ, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(calibrator.getAccelerometerSx(), sxa, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), sya, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), sza, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), mxya, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), mxza, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), myxa, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), myza, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), mzxa, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), mzya, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getInitialBiasX(), bgx, 0.0);
        assertEquals(calibrator.getInitialBiasY(), bgy, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), bgz, 0.0);

        final AngularSpeed angularSpeedX1 = calibrator
                .getInitialBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx,
                0.0);
        assertEquals(angularSpeedX1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator
                .getInitialBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy,
                0.0);
        assertEquals(angularSpeedY1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator
                .getInitialBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz,
                0.0);
        assertEquals(angularSpeedZ1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), bgx, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), bgy, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);

        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);

        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);

        assertSame(calibrator.getSequences(), sequences);

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(calibrator.getProgressDelta(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getPreliminarySubsetSize(),
                EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false,
                    new double[1], mg, gg, baArray, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias,
                    new Matrix(1, 3), gg,
                    baArray, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias,
                    new Matrix(3, 1), gg,
                    baArray, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias,
                    mg, new Matrix(1, 3),
                    baArray, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias,
                    mg, new Matrix(3, 1),
                    baArray, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias,
                    mg, gg, new double[1], ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias,
                    mg, gg, baArray,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bias,
                    mg, gg, baArray,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor16() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator(
                        sequences, false,
                        false,
                        bg, mg, gg, ba, ma);

        // check default values
        assertEquals(calibrator.getAccelerometerBiasX(),
                baX, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(),
                baY, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(),
                baZ, 0.0);
        final Acceleration accelerationX1 = calibrator
                .getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(),
                baX, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator
                .getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(),
                baY, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator
                .getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(),
                baZ, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(calibrator.getAccelerometerSx(), sxa, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), sya, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), sza, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), mxya, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), mxza, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), myxa, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), myza, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), mzxa, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), mzya, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getInitialBiasX(), bgx, 0.0);
        assertEquals(calibrator.getInitialBiasY(), bgy, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), bgz, 0.0);

        final AngularSpeed angularSpeedX1 = calibrator
                .getInitialBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx,
                0.0);
        assertEquals(angularSpeedX1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator
                .getInitialBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy,
                0.0);
        assertEquals(angularSpeedY1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator
                .getInitialBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz,
                0.0);
        assertEquals(angularSpeedZ1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), bgx, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), bgy, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);

        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);

        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);

        assertSame(calibrator.getSequences(), sequences);

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(calibrator.getProgressDelta(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getPreliminarySubsetSize(),
                EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false,
                    new Matrix(1, 1), mg,
                    gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false,
                    new Matrix(3, 3), mg,
                    gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg,
                    new Matrix(1, 3),
                    gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg,
                    new Matrix(3, 1),
                    gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg,
                    mg, new Matrix(1, 3),
                    ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg,
                    mg, new Matrix(3, 1),
                    ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg,
                    mg, gg, new Matrix(1, 1),
                    ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg,
                    mg, gg, new Matrix(3, 3),
                    ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg, mg, gg, ba,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg, mg, gg, ba,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor17() throws WrongSizeException {
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();

        final Matrix bg = generateBg();
        final Matrix mg = generateGeneralMg();
        final Matrix gg = generateGg();

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double[] bias = bg.getBuffer();

        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        final Matrix ba = generateBa();
        final Matrix ma = generateMa();
        final double[] baArray = ba.getBuffer();

        final double baX = ba.getElementAtIndex(0);
        final double baY = ba.getElementAtIndex(1);
        final double baZ = ba.getElementAtIndex(2);

        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator(
                        sequences, false,
                        false,
                        bg, mg, gg, ba, ma, this);

        // check default values
        assertEquals(calibrator.getAccelerometerBiasX(),
                baX, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(),
                baY, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(),
                baZ, 0.0);
        final Acceleration accelerationX1 = calibrator
                .getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(),
                baX, 0.0);
        assertEquals(accelerationX1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator
                .getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(),
                baY, 0.0);
        assertEquals(accelerationY1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator
                .getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(),
                baZ, 0.0);
        assertEquals(accelerationZ1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] ba1 = calibrator.getAccelerometerBias();
        assertArrayEquals(ba1, baArray, 0.0);
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);
        assertArrayEquals(ba1, ba2, 0.0);
        final Matrix ba1Matrix = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1Matrix, ba);
        final Matrix ba2Matrix = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2Matrix);
        assertEquals(ba1Matrix, ba2Matrix);
        assertEquals(calibrator.getAccelerometerSx(), sxa, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), sya, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), sza, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), mxya, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), mxza, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), myxa, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), myza, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), mzxa, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), mzya, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getInitialBiasX(), bgx, 0.0);
        assertEquals(calibrator.getInitialBiasY(), bgy, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), bgz, 0.0);

        final AngularSpeed angularSpeedX1 = calibrator
                .getInitialBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx,
                0.0);
        assertEquals(angularSpeedX1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);

        final AngularSpeed angularSpeedY1 = calibrator
                .getInitialBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy,
                0.0);
        assertEquals(angularSpeedY1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);

        final AngularSpeed angularSpeedZ1 = calibrator
                .getInitialBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz,
                0.0);
        assertEquals(angularSpeedZ1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);

        final AngularSpeedTriad initialBiasTriad1 = calibrator.getInitialBiasAsTriad();
        assertEquals(initialBiasTriad1.getValueX(), bgx, 0.0);
        assertEquals(initialBiasTriad1.getValueY(), bgy, 0.0);
        assertEquals(initialBiasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(initialBiasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad initialBiasTriad2 = new AngularSpeedTriad();
        calibrator.getInitialBiasAsTriad(initialBiasTriad2);
        assertEquals(initialBiasTriad1, initialBiasTriad2);

        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);

        final double[] bg1 = calibrator.getInitialBias();
        assertArrayEquals(bg1, bias, 0.0);
        final double[] bg2 = new double[3];
        calibrator.getInitialBias(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        final Matrix bg1Matrix = calibrator.getInitialBiasAsMatrix();
        assertEquals(bg1Matrix, bg);
        final Matrix bg2Matrix = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2Matrix);
        assertEquals(bg1Matrix, bg2Matrix);

        final Matrix mg1 = calibrator.getInitialMg();
        assertEquals(mg1, mg);
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);
        assertEquals(mg1, mg2);

        final Matrix gg1 = calibrator.getInitialGg();
        assertEquals(gg1, gg);
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);
        assertEquals(gg1, gg2);

        assertSame(calibrator.getSequences(), sequences);

        assertEquals(GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE,
                calibrator.getMeasurementOrSequenceType());
        assertTrue(calibrator.isOrderedMeasurementsOrSequencesRequired());
        assertFalse(calibrator.isQualityScoresRequired());
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 13);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());

        assertEquals(calibrator.getProgressDelta(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA,
                0.0);
        assertEquals(calibrator.getConfidence(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE,
                0.0);
        assertEquals(calibrator.getMaxIterations(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD,
                0.0);
        assertEquals(calibrator.getPreliminarySubsetSize(),
                EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);

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
        assertEquals(calibrator.getEstimatedChiSq(), 0.0, 0.0);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false,
                    new Matrix(1, 1), mg,
                    gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false,
                    new Matrix(3, 3), mg,
                    gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg,
                    new Matrix(1, 3),
                    gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg,
                    new Matrix(3, 1),
                    gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg,
                    mg, new Matrix(1, 3),
                    ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg,
                    mg, new Matrix(3, 1),
                    ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg,
                    mg, gg, new Matrix(1, 1),
                    ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg,
                    mg, gg, new Matrix(3, 3),
                    ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg, mg, gg, ba,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustEasyGyroscopeCalibrator(
                    sequences, false,
                    false, bg, mg, gg, ba,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testGetSetStopThreshold() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);

        // set new value
        calibrator.setStopThreshold(1.0);

        // check
        assertEquals(calibrator.getStopThreshold(), 1.0, 0.0);

        // Force IllegalArgumentException
        try {
            calibrator.setStopThreshold(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAccelerometerBiasX() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0,
                0.0);

        // set new value
        final Matrix ba = generateBa();
        final double bax = ba.getElementAtIndex(0);

        calibrator.setAccelerometerBiasX(bax);

        // check
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
    }

    @Test
    public void testGetSetAccelerometerBiasY() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0,
                0.0);

        // set new value
        final Matrix ba = generateBa();
        final double bay = ba.getElementAtIndex(1);

        calibrator.setAccelerometerBiasY(bay);

        // check
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
    }

    @Test
    public void testGetSetAccelerometerBiasZ() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0,
                0.0);

        // set new value
        final Matrix ba = generateBa();
        final double baz = ba.getElementAtIndex(2);

        calibrator.setAccelerometerBiasZ(baz);

        // check
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
    }

    @Test
    public void testGetSetAccelerometerBiasXAsAcceleration()
            throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        final Acceleration bax1 = calibrator
                .getAccelerometerBiasXAsAcceleration();
        assertEquals(bax1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bax1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration bax2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        // set new value
        final Matrix ba = generateBa();
        final double bax = ba.getElementAtIndex(0);
        final Acceleration bax3 = new Acceleration(bax,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setAccelerometerBiasX(bax3);

        // check
        final Acceleration bax4 = calibrator
                .getAccelerometerBiasXAsAcceleration();
        final Acceleration bax5 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(bax5);
        assertEquals(bax3, bax4);
        assertEquals(bax3, bax5);
    }

    @Test
    public void testGetSetAccelerometerBiasYAsAcceleration()
            throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        final Acceleration bay1 = calibrator
                .getAccelerometerBiasYAsAcceleration();
        assertEquals(bay1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bay1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration bay2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        // set new value
        final Matrix ba = generateBa();
        final double bay = ba.getElementAtIndex(1);
        final Acceleration bay3 = new Acceleration(bay,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setAccelerometerBiasY(bay3);

        // check
        final Acceleration bay4 = calibrator
                .getAccelerometerBiasYAsAcceleration();
        final Acceleration bay5 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(bay5);
        assertEquals(bay3, bay4);
        assertEquals(bay3, bay5);
    }

    @Test
    public void testGetSetAccelerometerBiasZAsAcceleration()
            throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        final Acceleration baz1 = calibrator
                .getAccelerometerBiasZAsAcceleration();
        assertEquals(baz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(baz1.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baz2 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        // set new value
        final Matrix ba = generateBa();
        final double baz = ba.getElementAtIndex(2);
        final Acceleration baz3 = new Acceleration(baz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setAccelerometerBiasZ(baz3);

        // check
        final Acceleration baz4 = calibrator
                .getAccelerometerBiasZAsAcceleration();
        final Acceleration baz5 = new Acceleration(0.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(baz5);
        assertEquals(baz3, baz4);
        assertEquals(baz3, baz5);
    }

    @Test
    public void testSetAccelerometerBias1() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0,
                0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0,
                0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0,
                0.0);

        // set new value
        final Matrix ba = generateBa();
        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        calibrator.setAccelerometerBias(bax, bay, baz);

        // check
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
    }

    @Test
    public void testSetAccelerometerBias2() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0,
                0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0,
                0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0,
                0.0);

        // set new value
        final Matrix ba = generateBa();
        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final Acceleration bax1 = new Acceleration(bax,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration bay1 = new Acceleration(bay,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration baz1 = new Acceleration(baz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setAccelerometerBias(bax1, bay1, baz1);

        // check
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
    }

    @Test
    public void testGetSetAccelerometerBias() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        final double[] ba1 = calibrator.getAccelerometerBias();
        final double[] ba2 = new double[3];
        calibrator.getAccelerometerBias(ba2);

        assertArrayEquals(ba1, new double[3], 0.0);
        assertArrayEquals(ba1, ba2, 0.0);

        // set new value
        final double[] ba3 = generateBa().getBuffer();
        calibrator.setAccelerometerBias(ba3);

        // check
        final double[] ba4 = calibrator.getAccelerometerBias();
        final double[] ba5 = new double[3];
        calibrator.getAccelerometerBias(ba5);

        assertArrayEquals(ba3, ba4, 0.0);
        assertArrayEquals(ba3, ba5, 0.0);

        // Force IllegalArgumentException
        try {
            calibrator.getAccelerometerBias(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setAccelerometerBias(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAccelerometerBiasAsMatrix()
            throws WrongSizeException, LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);

        // check
        assertEquals(ba1, new Matrix(3, 1));
        assertEquals(ba1, ba2);

        // set new value
        final Matrix ba3 = generateBa();
        calibrator.setAccelerometerBias(ba3);

        final Matrix ba4 = calibrator.getAccelerometerBiasAsMatrix();
        final Matrix ba5 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba5);

        assertEquals(ba3, ba4);
        assertEquals(ba3, ba5);
    }

    @Test
    public void testGetSetAccelerometerSx() throws WrongSizeException,
            LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerSx(), 0.0,
                0.0);

        // set new value
        final Matrix ma = generateMa();
        final double sxa = ma.getElementAt(0, 0);

        calibrator.setAccelerometerSx(sxa);

        // check
        assertEquals(calibrator.getAccelerometerSx(), sxa, 0.0);
    }

    @Test
    public void testGetSetAccelerometerSy() throws WrongSizeException,
            LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerSx(), 0.0,
                0.0);

        // set new value
        final Matrix ma = generateMa();
        final double sya = ma.getElementAt(1, 1);

        calibrator.setAccelerometerSy(sya);

        // check
        assertEquals(calibrator.getAccelerometerSy(), sya, 0.0);
    }

    @Test
    public void testGetSetAccelerometerSz() throws WrongSizeException,
            LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerSz(), 0.0,
                0.0);

        // set new value
        final Matrix ma = generateMa();
        final double sza = ma.getElementAt(2, 2);

        calibrator.setAccelerometerSz(sza);

        // check
        assertEquals(calibrator.getAccelerometerSz(), sza, 0.0);
    }

    @Test
    public void testGetSetAccelerometerMxy() throws WrongSizeException,
            LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerMxy(), 0.0,
                0.0);

        // set new value
        final Matrix ma = generateMa();
        final double mxya = ma.getElementAt(0, 1);

        calibrator.setAccelerometerMxy(mxya);

        // check
        assertEquals(calibrator.getAccelerometerMxy(), mxya, 0.0);
    }

    @Test
    public void testGetSetAccelerometerMxz() throws WrongSizeException,
            LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerMxz(), 0.0,
                0.0);

        // set new value
        final Matrix ma = generateMa();
        final double mxza = ma.getElementAt(0, 2);

        calibrator.setAccelerometerMxz(mxza);

        // check
        assertEquals(calibrator.getAccelerometerMxz(), mxza, 0.0);
    }

    @Test
    public void testGetSetAccelerometerMyx() throws WrongSizeException,
            LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerMyx(), 0.0,
                0.0);

        // set new value
        final Matrix ma = generateMa();
        final double myxa = ma.getElementAt(1, 0);

        calibrator.setAccelerometerMyx(myxa);

        // check
        assertEquals(calibrator.getAccelerometerMyx(), myxa, 0.0);
    }

    @Test
    public void testGetSetAccelerometerMyz() throws WrongSizeException,
            LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerMyz(), 0.0,
                0.0);

        // set new value
        final Matrix ma = generateMa();
        final double myza = ma.getElementAt(1, 2);

        calibrator.setAccelerometerMyz(myza);

        // check
        assertEquals(calibrator.getAccelerometerMyz(), myza, 0.0);
    }

    @Test
    public void testGetSetAccelerometerMzx() throws WrongSizeException,
            LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerMzx(), 0.0,
                0.0);

        // set new value
        final Matrix ma = generateMa();
        final double mzxa = ma.getElementAt(2, 0);

        calibrator.setAccelerometerMzx(mzxa);

        // check
        assertEquals(calibrator.getAccelerometerMzx(), mzxa, 0.0);
    }

    @Test
    public void testGetSetAccelerometerMzy() throws WrongSizeException,
            LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerMzy(), 0.0,
                0.0);

        // set new value
        final Matrix ma = generateMa();
        final double mzya = ma.getElementAt(2, 1);

        calibrator.setAccelerometerMzy(mzya);

        // check
        assertEquals(calibrator.getAccelerometerMzy(), mzya, 0.0);
    }

    @Test
    public void testSetAccelerometerScalingFactors()
            throws WrongSizeException, LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getAccelerometerSx(), 0.0,
                0.0);
        assertEquals(calibrator.getAccelerometerSy(), 0.0,
                0.0);
        assertEquals(calibrator.getAccelerometerSz(), 0.0,
                0.0);

        // set new values
        final Matrix ma = generateMa();
        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);

        calibrator.setAccelerometerScalingFactors(sxa, sya, sza);

        // check
        assertEquals(calibrator.getAccelerometerSx(), sxa,
                0.0);
        assertEquals(calibrator.getAccelerometerSy(), sya,
                0.0);
        assertEquals(calibrator.getAccelerometerSz(), sza,
                0.0);
    }

    @Test
    public void testSetAccelerometerCrossCouplingErrors()
            throws WrongSizeException, LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getAccelerometerMxy(), 0.0,
                0.0);
        assertEquals(calibrator.getAccelerometerMxz(), 0.0,
                0.0);
        assertEquals(calibrator.getAccelerometerMyx(), 0.0,
                0.0);
        assertEquals(calibrator.getAccelerometerMyz(), 0.0,
                0.0);
        assertEquals(calibrator.getAccelerometerMzx(), 0.0,
                0.0);
        assertEquals(calibrator.getAccelerometerMzy(), 0.0,
                0.0);

        // set new values
        final Matrix ma = generateMa();
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        calibrator.setAccelerometerCrossCouplingErrors(mxya, mxza,
                myxa, myza, mzxa, mzya);

        // check
        assertEquals(calibrator.getAccelerometerMxy(), mxya,
                0.0);
        assertEquals(calibrator.getAccelerometerMxz(), mxza,
                0.0);
        assertEquals(calibrator.getAccelerometerMyx(), myxa,
                0.0);
        assertEquals(calibrator.getAccelerometerMyz(), myza,
                0.0);
        assertEquals(calibrator.getAccelerometerMzx(), mzxa,
                0.0);
        assertEquals(calibrator.getAccelerometerMzy(), mzya,
                0.0);
    }

    @Test
    public void testSetAccelerometerScalingFactorsAndCrossCouplingErrors()
            throws WrongSizeException, LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getAccelerometerSx(), 0.0,
                0.0);
        assertEquals(calibrator.getAccelerometerSy(), 0.0,
                0.0);
        assertEquals(calibrator.getAccelerometerSz(), 0.0,
                0.0);
        assertEquals(calibrator.getAccelerometerMxy(), 0.0,
                0.0);
        assertEquals(calibrator.getAccelerometerMxz(), 0.0,
                0.0);
        assertEquals(calibrator.getAccelerometerMyx(), 0.0,
                0.0);
        assertEquals(calibrator.getAccelerometerMyz(), 0.0,
                0.0);
        assertEquals(calibrator.getAccelerometerMzx(), 0.0,
                0.0);
        assertEquals(calibrator.getAccelerometerMzy(), 0.0,
                0.0);

        // set new values
        final Matrix ma = generateMa();
        final double sxa = ma.getElementAt(0, 0);
        final double sya = ma.getElementAt(1, 1);
        final double sza = ma.getElementAt(2, 2);
        final double mxya = ma.getElementAt(0, 1);
        final double mxza = ma.getElementAt(0, 2);
        final double myxa = ma.getElementAt(1, 0);
        final double myza = ma.getElementAt(1, 2);
        final double mzxa = ma.getElementAt(2, 0);
        final double mzya = ma.getElementAt(2, 1);

        calibrator.setAccelerometerScalingFactorsAndCrossCouplingErrors(
                sxa, sya, sza, mxya, mxza,
                myxa, myza, mzxa, mzya);

        // check
        assertEquals(calibrator.getAccelerometerSx(), sxa,
                0.0);
        assertEquals(calibrator.getAccelerometerSy(), sya,
                0.0);
        assertEquals(calibrator.getAccelerometerSz(), sza,
                0.0);
        assertEquals(calibrator.getAccelerometerMxy(), mxya,
                0.0);
        assertEquals(calibrator.getAccelerometerMxz(), mxza,
                0.0);
        assertEquals(calibrator.getAccelerometerMyx(), myxa,
                0.0);
        assertEquals(calibrator.getAccelerometerMyz(), myza,
                0.0);
        assertEquals(calibrator.getAccelerometerMzx(), mzxa,
                0.0);
        assertEquals(calibrator.getAccelerometerMzy(), mzya,
                0.0);
    }

    @Test
    public void testGetSetAccelerometerMa() throws WrongSizeException,
            LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check initial value
        assertEquals(calibrator.getAccelerometerMa(),
                new Matrix(3, 3));
        final Matrix ma1 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma1);
        assertEquals(ma1, new Matrix(3, 3));

        // set new value
        final Matrix ma2 = generateMa();
        calibrator.setAccelerometerMa(ma2);

        // check
        assertEquals(ma2, calibrator.getAccelerometerMa());
        final Matrix ma3 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma3);
        assertEquals(ma2, ma3);

        // Force IllegalArgumentException
        try {
            calibrator.getAccelerometerMa(new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.getAccelerometerMa(new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setAccelerometerMa(new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setAccelerometerMa(new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetInitialBiasX() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double bgx = bg.getElementAtIndex(0);

        calibrator.setInitialBiasX(bgx);

        // check
        assertEquals(calibrator.getInitialBiasX(), bgx, 0.0);
    }

    @Test
    public void testGetSetInitialBiasY() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double bgy = bg.getElementAtIndex(1);

        calibrator.setInitialBiasY(bgy);

        // check
        assertEquals(calibrator.getInitialBiasY(), bgy, 0.0);
    }

    @Test
    public void testGetSetInitialBiasZ() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double bgz = bg.getElementAtIndex(2);

        calibrator.setInitialBiasZ(bgz);

        // check
        assertEquals(calibrator.getInitialBiasZ(), bgz, 0.0);
    }

    @Test
    public void testGetSetInitialBiasAngularSpeedX()
            throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        final AngularSpeed bgx1 = calibrator.getInitialBiasAngularSpeedX();
        assertEquals(bgx1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgx1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

        final AngularSpeed bgx2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bgx2);
        assertEquals(bgx1, bgx2);

        // set new value
        final Matrix bg = generateBg();
        final double bgx = bg.getElementAtIndex(0);
        final AngularSpeed bgx3 = new AngularSpeed(bgx,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.setInitialBiasX(bgx3);

        // check
        final AngularSpeed bgx4 = calibrator.getInitialBiasAngularSpeedX();
        final AngularSpeed bgx5 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedX(bgx5);

        assertEquals(bgx3, bgx4);
        assertEquals(bgx3, bgx5);
    }

    @Test
    public void testGetSetInitialBiasAngularSpeedY()
            throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        final AngularSpeed bgy1 = calibrator.getInitialBiasAngularSpeedY();
        assertEquals(bgy1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgy1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

        final AngularSpeed bgy2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(bgy2);
        assertEquals(bgy1, bgy2);

        // set new value
        final Matrix bg = generateBg();
        final double bgy = bg.getElementAtIndex(1);
        final AngularSpeed bgy3 = new AngularSpeed(bgy,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.setInitialBiasY(bgy3);

        // check
        final AngularSpeed bgy4 = calibrator.getInitialBiasAngularSpeedY();
        final AngularSpeed bgy5 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedY(bgy5);

        assertEquals(bgy3, bgy4);
        assertEquals(bgy3, bgy5);
    }

    @Test
    public void testGetSetInitialBiasAngularSpeedZ()
            throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        final AngularSpeed bgz1 = calibrator.getInitialBiasAngularSpeedZ();
        assertEquals(bgz1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(bgz1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);

        final AngularSpeed bgz2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bgz2);
        assertEquals(bgz1, bgz2);

        // set new value
        final Matrix bg = generateBg();
        final double bgz = bg.getElementAtIndex(2);
        final AngularSpeed bgz3 = new AngularSpeed(bgz,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.setInitialBiasZ(bgz3);

        // check
        final AngularSpeed bgz4 = calibrator.getInitialBiasAngularSpeedZ();
        final AngularSpeed bgz5 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getInitialBiasAngularSpeedZ(bgz5);

        assertEquals(bgz3, bgz4);
        assertEquals(bgz3, bgz5);
    }

    @Test
    public void testSetInitialBias1() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);

        // set new values
        final Matrix bg = generateBg();
        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        calibrator.setInitialBias(bgx, bgy, bgz);

        // check
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
    }

    @Test
    public void testSetInitialBias2() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getInitialBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getInitialBiasZ(), 0.0, 0.0);

        // set new values
        final Matrix bg = generateBg();
        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final AngularSpeed bgx1 = new AngularSpeed(bgx,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy1 = new AngularSpeed(bgy,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz1 = new AngularSpeed(bgz,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setInitialBias(bgx1, bgy1, bgz1);

        // check
        assertEquals(bgx, calibrator.getInitialBiasX(), 0.0);
        assertEquals(bgy, calibrator.getInitialBiasY(), 0.0);
        assertEquals(bgz, calibrator.getInitialBiasZ(), 0.0);
    }

    @Test
    public void testGetSetInitialBiasAsTriad() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

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
    public void testGetSetInitialSx() throws WrongSizeException,
            LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateGeneralMg();
        final double sx = mg.getElementAt(0, 0);

        calibrator.setInitialSx(sx);

        // check
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
    }

    @Test
    public void testGetSetInitialSy() throws WrongSizeException,
            LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateGeneralMg();
        final double sy = mg.getElementAt(1, 1);

        calibrator.setInitialSy(sy);

        // check
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
    }

    @Test
    public void testGetSetInitialSz() throws WrongSizeException,
            LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateGeneralMg();
        final double sz = mg.getElementAt(2, 2);

        calibrator.setInitialSz(sz);

        // check
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
    }

    @Test
    public void testGetSetInitialMxy() throws WrongSizeException,
            LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateGeneralMg();
        final double mxy = mg.getElementAt(0, 1);

        calibrator.setInitialMxy(mxy);

        // check
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
    }

    @Test
    public void testGetSetInitialMxz() throws WrongSizeException,
            LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateGeneralMg();
        final double mxz = mg.getElementAt(0, 2);

        calibrator.setInitialMxz(mxz);

        // check
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
    }

    @Test
    public void testGetSetInitialMyx() throws WrongSizeException,
            LockedException {

        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateGeneralMg();
        final double myx = mg.getElementAt(1, 0);

        calibrator.setInitialMyx(myx);

        // check
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
    }

    @Test
    public void testGetSetInitialMyz() throws WrongSizeException,
            LockedException {

        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateGeneralMg();
        final double myz = mg.getElementAt(1, 2);

        calibrator.setInitialMyz(myz);

        // check
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
    }

    @Test
    public void testGetSetInitialMzx() throws WrongSizeException,
            LockedException {

        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateGeneralMg();
        final double mzx = mg.getElementAt(2, 0);

        calibrator.setInitialMzx(mzx);

        // check
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
    }

    @Test
    public void testGetSetInitialMzy() throws WrongSizeException,
            LockedException {

        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateGeneralMg();
        final double mzy = mg.getElementAt(2, 1);

        calibrator.setInitialMzy(mzy);

        // check
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
    }

    @Test
    public void testSetInitialScalingFactors() throws WrongSizeException,
            LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);

        // set new values
        final Matrix mg = generateGeneralMg();
        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);

        calibrator.setInitialScalingFactors(sx, sy, sz);

        // check
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
    }

    @Test
    public void testSetInitialCrossCouplingErrors()
            throws WrongSizeException, LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        // set new values
        final Matrix mg = generateGeneralMg();
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        calibrator.setInitialCrossCouplingErrors(mxy, mxz, myx, myz,
                mzx, mzy);

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
            throws WrongSizeException, LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

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
        final Matrix mg = generateGeneralMg();
        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

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
    public void testGetSetInitialBias() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check initial value
        final double[] bg1 = calibrator.getInitialBias();
        final double[] bg2 = new double[3];
        calibrator.getInitialBias(bg2);

        assertArrayEquals(bg1, new double[3], 0.0);
        assertArrayEquals(bg1, bg2, 0.0);

        // set new value
        final double[] bg3 = generateBg().getBuffer();
        calibrator.setInitialBias(bg3);

        // check
        final double[] bg4 = calibrator.getInitialBias();
        final double[] bg5 = new double[3];
        calibrator.getInitialBias(bg5);

        assertArrayEquals(bg3, bg4, 0.0);
        assertArrayEquals(bg3, bg5, 0.0);


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
    public void testGetSetInitialBiasAsMatrix()
            throws WrongSizeException, LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check initial values
        final Matrix bg1 = calibrator.getInitialBiasAsMatrix();
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg2);

        assertEquals(bg1, new Matrix(3, 1));
        assertEquals(bg1, bg2);

        // set new value
        final Matrix bg3 = generateBg();
        calibrator.setInitialBias(bg3);

        // check
        final Matrix bg4 = calibrator.getInitialBiasAsMatrix();
        final Matrix bg5 = new Matrix(3, 1);
        calibrator.getInitialBiasAsMatrix(bg5);

        assertEquals(bg3, bg4);
        assertEquals(bg3, bg5);

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
    public void testGetSetInitialMg() throws WrongSizeException,
            LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check initial value
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);

        assertEquals(mg1, new Matrix(3, 3));
        assertEquals(mg1, mg2);

        // set new value
        final Matrix mg3 = generateGeneralMg();
        calibrator.setInitialMg(mg3);

        // check
        final Matrix mg4 = calibrator.getInitialMg();
        final Matrix mg5 = new Matrix(3, 3);
        calibrator.getInitialMg(mg5);

        assertEquals(mg3, mg4);
        assertEquals(mg3, mg5);


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
    public void testGetSetInitialGg() throws WrongSizeException,
            LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check initial value
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);

        assertEquals(gg1, new Matrix(3, 3));
        assertEquals(gg1, gg2);

        // set new value
        final Matrix gg3 = generateGg();
        calibrator.setInitialGg(gg3);

        // check
        final Matrix gg4 = calibrator.getInitialGg();
        final Matrix gg5 = new Matrix(3, 3);
        calibrator.getInitialGg(gg5);

        assertEquals(gg3, gg4);
        assertEquals(gg3, gg5);


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
    public void testGetSetSequences() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check initial value
        assertNull(calibrator.getSequences());

        // set new value
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences =
                Collections.emptyList();
        calibrator.setSequences(sequences);

        // check
        assertSame(calibrator.getSequences(), sequences);
    }

    @Test
    public void testIsSetCommonAxisUsed() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check initial value
        assertTrue(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(false);

        // check
        assertFalse(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testIsSetGDependentCrossBiasesEstimated()
            throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check initial value
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());

        // set new value
        calibrator.setGDependentCrossBiasesEstimated(false);

        // check
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
    }

    @Test
    public void tetGetSetListener() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check initial value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testGetMinimumRequiredMeasurementsOrSequences() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check initial value
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 19);
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());

        // set new value
        calibrator.setGDependentCrossBiasesEstimated(false);

        // check
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 10);
        assertTrue(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());

        // set new value
        calibrator.setCommonAxisUsed(false);

        // check
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 13);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());

        // set new value
        calibrator.setGDependentCrossBiasesEstimated(true);

        // check
        assertEquals(calibrator.getMinimumRequiredMeasurementsOrSequences(), 22);
        assertFalse(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
    }

    @Test
    public void testIsReady() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        assertFalse(calibrator.isReady());

        // set not enough sequences
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences1 =
                Collections.emptyList();
        calibrator.setSequences(sequences1);

        // check
        assertFalse(calibrator.isReady());

        // set enough sequences
        final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences2 =
                new ArrayList<>();
        for (int i = 0; i < calibrator.getMinimumRequiredMeasurementsOrSequences(); i++) {
            sequences2.add(new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>());
        }
        calibrator.setSequences(sequences2);

        // check
        assertTrue(calibrator.isReady());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getProgressDelta(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);

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
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getConfidence(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);

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
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getMaxIterations(),
                LMedSRobustEasyGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);

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
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertTrue(calibrator.isResultRefined());

        // set new value
        calibrator.setResultRefined(false);

        // check
        assertFalse(calibrator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertTrue(calibrator.isCovarianceKept());

        // set new value
        calibrator.setCovarianceKept(false);

        // check
        assertFalse(calibrator.isCovarianceKept());
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertNull(calibrator.getQualityScores());

        // set new value
        calibrator.setQualityScores(new double[3]);

        // check
        assertNull(calibrator.getQualityScores());
    }

    @Test
    public void testGetSetPreliminarySubsetSize() throws LockedException {
        final LMedSRobustEasyGyroscopeCalibrator calibrator =
                new LMedSRobustEasyGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getPreliminarySubsetSize(),
                EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_GENERAL_AND_CROSS_BIASES);

        // set new value
        calibrator.setPreliminarySubsetSize(20);

        // check
        assertEquals(calibrator.getPreliminarySubsetSize(), 20);

        // force IllegalArgumentException
        try {
            calibrator.setPreliminarySubsetSize(6);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testCalibrateCommonAxisAndGDependentCrossBiasesDisabledAndNoInlierNoise()
            throws WrongSizeException, InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateCommonAxisMg();
            final Matrix gg = new Matrix(3, 3);
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
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final int m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final double roll = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final CoordinateTransformation nedC = new CoordinateTransformation(
                        roll, pitch, yaw, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final Quaternion beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
                final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame);

                final BodyKinematics trueBeforeGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                                ecefFrame, ecefFrame);
                final BodyKinematics measuredBeforeGravityKinematics =
                        BodyKinematicsGenerator.generate(
                                TIME_INTERVAL_SECONDS,
                                trueBeforeGravityKinematics,
                                errorsInlier, random);

                final double beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final double beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final double beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final double deltaRoll = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));
                final double deltaPitch = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));
                final double deltaYaw = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));

                NEDFrame oldNedFrame = new NEDFrame(nedFrame);
                NEDFrame newNedFrame = new NEDFrame();
                ECEFFrame oldEcefFrame = new ECEFFrame();
                ECEFFrame newEcefFrame = new ECEFFrame();
                double oldRoll = roll - deltaRoll;
                double oldPitch = pitch - deltaPitch;
                double oldYaw = yaw - deltaYaw;

                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                        new BodyKinematicsSequence<>();
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        new BodyKinematicsSequence<>();
                sequence.setBeforeMeanSpecificForceCoordinates(
                        beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList =
                        new ArrayList<>();
                final List<StandardDeviationTimedBodyKinematics> measuredTimedKinematicsList =
                        new ArrayList<>();
                final boolean sequenceCanHaveOutliers =
                        randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                for (int j = 0; j < m; j++) {
                    final double newRoll = oldRoll + deltaRoll;
                    final double newPitch = oldPitch + deltaPitch;
                    final double newYaw = oldYaw + deltaYaw;
                    final CoordinateTransformation newNedC =
                            new CoordinateTransformation(
                                    newRoll, newPitch, newYaw,
                                    FrameType.BODY_FRAME,
                                    FrameType.LOCAL_NAVIGATION_FRAME);
                    final NEDPosition newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                            .estimateKinematicsAndReturnNew(
                                    TIME_INTERVAL_SECONDS, newEcefFrame,
                                    oldEcefFrame);

                    // apply known calibration parameters to distort ground-truth and generate a
                    // measured kinematics sample
                    final BodyKinematics measuredKinematics;
                    if (sequenceCanHaveOutliers &&
                            randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                        // outlier
                        measuredKinematics = BodyKinematicsGenerator
                                .generate(TIME_INTERVAL_SECONDS,
                                        trueKinematics, errorsOutlier,
                                        random);

                    } else {
                        // inlier
                        measuredKinematics = BodyKinematicsGenerator
                                .generate(TIME_INTERVAL_SECONDS,
                                        trueKinematics, errorsInlier,
                                        random);
                    }

                    final StandardDeviationTimedBodyKinematics trueTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(
                                    trueKinematics, timestampSeconds,
                                    specificForceStandardDeviation,
                                    angularRateStandardDeviation);

                    final StandardDeviationTimedBodyKinematics measuredTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(
                                    measuredKinematics, timestampSeconds,
                                    specificForceStandardDeviation,
                                    angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final Quaternion afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(
                        trueSequence, beforeQ, afterQ);

                final CoordinateTransformation newNedC =
                        new CoordinateTransformation(
                                afterQ.asInhomogeneousMatrix(),
                                FrameType.BODY_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final BodyKinematics trueAfterGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                                newEcefFrame, newEcefFrame);
                final BodyKinematics measuredAfterGravityKinematics =
                        BodyKinematicsGenerator.generate(
                                TIME_INTERVAL_SECONDS,
                                trueAfterGravityKinematics,
                                errorsInlier, random);

                final double afterMeanFx = measuredAfterGravityKinematics.getFx();
                final double afterMeanFy = measuredAfterGravityKinematics.getFy();
                final double afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(
                        afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final LMedSRobustEasyGyroscopeCalibrator calibrator =
                    new LMedSRobustEasyGyroscopeCalibrator(sequences,
                            true, false,
                            bg, mg, gg, ba, ma, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);
            assertEquals(mCalibrateNextIteration, 0);
            assertEquals(mCalibrateProgressChange, 0);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

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
    public void testCalibrateGeneralAndGDependentCrossBiasesDisabledAndNoInlierNoise()
            throws WrongSizeException, InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateGeneralMg();
            final Matrix gg = new Matrix(3, 3);
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
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final int m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final double roll = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final CoordinateTransformation nedC = new CoordinateTransformation(
                        roll, pitch, yaw, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final Quaternion beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
                final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame);

                final BodyKinematics trueBeforeGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                                ecefFrame, ecefFrame);
                final BodyKinematics measuredBeforeGravityKinematics =
                        BodyKinematicsGenerator.generate(
                                TIME_INTERVAL_SECONDS,
                                trueBeforeGravityKinematics,
                                errorsInlier, random);

                final double beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final double beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final double beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final double deltaRoll = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));
                final double deltaPitch = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));
                final double deltaYaw = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));

                NEDFrame oldNedFrame = new NEDFrame(nedFrame);
                NEDFrame newNedFrame = new NEDFrame();
                ECEFFrame oldEcefFrame = new ECEFFrame();
                ECEFFrame newEcefFrame = new ECEFFrame();
                double oldRoll = roll - deltaRoll;
                double oldPitch = pitch - deltaPitch;
                double oldYaw = yaw - deltaYaw;

                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                        new BodyKinematicsSequence<>();
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        new BodyKinematicsSequence<>();
                sequence.setBeforeMeanSpecificForceCoordinates(
                        beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList =
                        new ArrayList<>();
                final List<StandardDeviationTimedBodyKinematics> measuredTimedKinematicsList =
                        new ArrayList<>();
                final boolean sequenceCanHaveOutliers =
                        randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                for (int j = 0; j < m; j++) {
                    final double newRoll = oldRoll + deltaRoll;
                    final double newPitch = oldPitch + deltaPitch;
                    final double newYaw = oldYaw + deltaYaw;
                    final CoordinateTransformation newNedC =
                            new CoordinateTransformation(
                                    newRoll, newPitch, newYaw,
                                    FrameType.BODY_FRAME,
                                    FrameType.LOCAL_NAVIGATION_FRAME);
                    final NEDPosition newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                            .estimateKinematicsAndReturnNew(
                                    TIME_INTERVAL_SECONDS, newEcefFrame,
                                    oldEcefFrame);

                    // apply known calibration parameters to distort ground-truth and generate a
                    // measured kinematics sample
                    final BodyKinematics measuredKinematics;
                    if (sequenceCanHaveOutliers &&
                            randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                        // outlier
                        measuredKinematics = BodyKinematicsGenerator
                                .generate(TIME_INTERVAL_SECONDS,
                                        trueKinematics, errorsOutlier,
                                        random);

                    } else {
                        // inlier
                        measuredKinematics = BodyKinematicsGenerator
                                .generate(TIME_INTERVAL_SECONDS,
                                        trueKinematics, errorsInlier,
                                        random);
                    }

                    final StandardDeviationTimedBodyKinematics trueTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(
                                    trueKinematics, timestampSeconds,
                                    specificForceStandardDeviation,
                                    angularRateStandardDeviation);

                    final StandardDeviationTimedBodyKinematics measuredTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(
                                    measuredKinematics, timestampSeconds,
                                    specificForceStandardDeviation,
                                    angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final Quaternion afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(
                        trueSequence, beforeQ, afterQ);

                final CoordinateTransformation newNedC =
                        new CoordinateTransformation(
                                afterQ.asInhomogeneousMatrix(),
                                FrameType.BODY_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final BodyKinematics trueAfterGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                                newEcefFrame, newEcefFrame);
                final BodyKinematics measuredAfterGravityKinematics =
                        BodyKinematicsGenerator.generate(
                                TIME_INTERVAL_SECONDS,
                                trueAfterGravityKinematics,
                                errorsInlier, random);

                final double afterMeanFx = measuredAfterGravityKinematics.getFx();
                final double afterMeanFy = measuredAfterGravityKinematics.getFy();
                final double afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(
                        afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final LMedSRobustEasyGyroscopeCalibrator calibrator =
                    new LMedSRobustEasyGyroscopeCalibrator(sequences,
                            false, false,
                            bg, mg, gg, ba, ma, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);
            assertEquals(mCalibrateNextIteration, 0);
            assertEquals(mCalibrateProgressChange, 0);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

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
    public void testCalibrateCommonAxisAndGDependentCrossBiasesEnabledAndNoInlierNoise()
            throws WrongSizeException, InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateCommonAxisMg();
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
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final int m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final double roll = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final CoordinateTransformation nedC = new CoordinateTransformation(
                        roll, pitch, yaw, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final Quaternion beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
                final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame);

                final BodyKinematics trueBeforeGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                                ecefFrame, ecefFrame);
                final BodyKinematics measuredBeforeGravityKinematics =
                        BodyKinematicsGenerator.generate(
                                TIME_INTERVAL_SECONDS,
                                trueBeforeGravityKinematics,
                                errorsInlier, random);

                final double beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final double beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final double beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final double deltaRoll = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));
                final double deltaPitch = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));
                final double deltaYaw = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));

                NEDFrame oldNedFrame = new NEDFrame(nedFrame);
                NEDFrame newNedFrame = new NEDFrame();
                ECEFFrame oldEcefFrame = new ECEFFrame();
                ECEFFrame newEcefFrame = new ECEFFrame();
                double oldRoll = roll - deltaRoll;
                double oldPitch = pitch - deltaPitch;
                double oldYaw = yaw - deltaYaw;

                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                        new BodyKinematicsSequence<>();
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        new BodyKinematicsSequence<>();
                sequence.setBeforeMeanSpecificForceCoordinates(
                        beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList =
                        new ArrayList<>();
                final List<StandardDeviationTimedBodyKinematics> measuredTimedKinematicsList =
                        new ArrayList<>();
                final boolean sequenceCanHaveOutliers =
                        randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                for (int j = 0; j < m; j++) {
                    final double newRoll = oldRoll + deltaRoll;
                    final double newPitch = oldPitch + deltaPitch;
                    final double newYaw = oldYaw + deltaYaw;
                    final CoordinateTransformation newNedC =
                            new CoordinateTransformation(
                                    newRoll, newPitch, newYaw,
                                    FrameType.BODY_FRAME,
                                    FrameType.LOCAL_NAVIGATION_FRAME);
                    final NEDPosition newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                            .estimateKinematicsAndReturnNew(
                                    TIME_INTERVAL_SECONDS, newEcefFrame,
                                    oldEcefFrame);

                    // apply known calibration parameters to distort ground-truth and generate a
                    // measured kinematics sample
                    final BodyKinematics measuredKinematics;
                    if (sequenceCanHaveOutliers &&
                            randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                        // outlier
                        measuredKinematics = BodyKinematicsGenerator
                                .generate(TIME_INTERVAL_SECONDS,
                                        trueKinematics, errorsOutlier,
                                        random);

                    } else {
                        // inlier
                        measuredKinematics = BodyKinematicsGenerator
                                .generate(TIME_INTERVAL_SECONDS,
                                        trueKinematics, errorsInlier,
                                        random);
                    }

                    final StandardDeviationTimedBodyKinematics trueTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(
                                    trueKinematics, timestampSeconds,
                                    specificForceStandardDeviation,
                                    angularRateStandardDeviation);

                    final StandardDeviationTimedBodyKinematics measuredTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(
                                    measuredKinematics, timestampSeconds,
                                    specificForceStandardDeviation,
                                    angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final Quaternion afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(
                        trueSequence, beforeQ, afterQ);

                final CoordinateTransformation newNedC =
                        new CoordinateTransformation(
                                afterQ.asInhomogeneousMatrix(),
                                FrameType.BODY_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final BodyKinematics trueAfterGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                                newEcefFrame, newEcefFrame);
                final BodyKinematics measuredAfterGravityKinematics =
                        BodyKinematicsGenerator.generate(
                                TIME_INTERVAL_SECONDS,
                                trueAfterGravityKinematics,
                                errorsInlier, random);

                final double afterMeanFx = measuredAfterGravityKinematics.getFx();
                final double afterMeanFy = measuredAfterGravityKinematics.getFy();
                final double afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(
                        afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final LMedSRobustEasyGyroscopeCalibrator calibrator =
                    new LMedSRobustEasyGyroscopeCalibrator(sequences,
                            true,
                            true,
                            bg, mg, gg, ba, ma, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);
            assertEquals(mCalibrateNextIteration, 0);
            assertEquals(mCalibrateProgressChange, 0);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

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
            if (!mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisAndGDependantCrossBiasesCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(calibrator.getEstimatedChiSq(), 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralAndGDependentCrossBiasesEnabledAndNoInlierNoise()
            throws WrongSizeException, InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateGeneralMg();
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
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final int m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final double roll = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final CoordinateTransformation nedC = new CoordinateTransformation(
                        roll, pitch, yaw, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final Quaternion beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
                final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame);

                final BodyKinematics trueBeforeGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                                ecefFrame, ecefFrame);
                final BodyKinematics measuredBeforeGravityKinematics =
                        BodyKinematicsGenerator.generate(
                                TIME_INTERVAL_SECONDS,
                                trueBeforeGravityKinematics,
                                errorsInlier, random);

                final double beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final double beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final double beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final double deltaRoll = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));
                final double deltaPitch = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));
                final double deltaYaw = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));

                NEDFrame oldNedFrame = new NEDFrame(nedFrame);
                NEDFrame newNedFrame = new NEDFrame();
                ECEFFrame oldEcefFrame = new ECEFFrame();
                ECEFFrame newEcefFrame = new ECEFFrame();
                double oldRoll = roll - deltaRoll;
                double oldPitch = pitch - deltaPitch;
                double oldYaw = yaw - deltaYaw;

                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                        new BodyKinematicsSequence<>();
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        new BodyKinematicsSequence<>();
                sequence.setBeforeMeanSpecificForceCoordinates(
                        beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList =
                        new ArrayList<>();
                final List<StandardDeviationTimedBodyKinematics> measuredTimedKinematicsList =
                        new ArrayList<>();
                final boolean sequenceCanHaveOutliers =
                        randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                for (int j = 0; j < m; j++) {
                    final double newRoll = oldRoll + deltaRoll;
                    final double newPitch = oldPitch + deltaPitch;
                    final double newYaw = oldYaw + deltaYaw;
                    final CoordinateTransformation newNedC =
                            new CoordinateTransformation(
                                    newRoll, newPitch, newYaw,
                                    FrameType.BODY_FRAME,
                                    FrameType.LOCAL_NAVIGATION_FRAME);
                    final NEDPosition newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                            .estimateKinematicsAndReturnNew(
                                    TIME_INTERVAL_SECONDS, newEcefFrame,
                                    oldEcefFrame);

                    // apply known calibration parameters to distort ground-truth and generate a
                    // measured kinematics sample
                    final BodyKinematics measuredKinematics;
                    if (sequenceCanHaveOutliers &&
                            randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                        // outlier
                        measuredKinematics = BodyKinematicsGenerator
                                .generate(TIME_INTERVAL_SECONDS,
                                        trueKinematics, errorsOutlier,
                                        random);

                    } else {
                        // inlier
                        measuredKinematics = BodyKinematicsGenerator
                                .generate(TIME_INTERVAL_SECONDS,
                                        trueKinematics, errorsInlier,
                                        random);
                    }

                    final StandardDeviationTimedBodyKinematics trueTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(
                                    trueKinematics, timestampSeconds,
                                    specificForceStandardDeviation,
                                    angularRateStandardDeviation);

                    final StandardDeviationTimedBodyKinematics measuredTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(
                                    measuredKinematics, timestampSeconds,
                                    specificForceStandardDeviation,
                                    angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final Quaternion afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(
                        trueSequence, beforeQ, afterQ);

                final CoordinateTransformation newNedC =
                        new CoordinateTransformation(
                                afterQ.asInhomogeneousMatrix(),
                                FrameType.BODY_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final BodyKinematics trueAfterGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                                newEcefFrame, newEcefFrame);
                final BodyKinematics measuredAfterGravityKinematics =
                        BodyKinematicsGenerator.generate(
                                TIME_INTERVAL_SECONDS,
                                trueAfterGravityKinematics,
                                errorsInlier, random);

                final double afterMeanFx = measuredAfterGravityKinematics.getFx();
                final double afterMeanFy = measuredAfterGravityKinematics.getFy();
                final double afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(
                        afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final LMedSRobustEasyGyroscopeCalibrator calibrator =
                    new LMedSRobustEasyGyroscopeCalibrator(sequences,
                            false,
                            true,
                            bg, mg, gg, ba, ma, this);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);
            assertEquals(mCalibrateNextIteration, 0);
            assertEquals(mCalibrateProgressChange, 0);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

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
            if (!mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralAndGDependantCrossBiasesCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(calibrator.getEstimatedChiSq(), 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateCommonAxisAndGDependentCrossBiasesDisabledWithInlierNoise()
            throws WrongSizeException, InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateCommonAxisMg();
            final Matrix gg = new Matrix(3, 3);
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
            final IMUErrors noErrorsInlier = new IMUErrors(ba, bg, ma, mg, gg,
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
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final int m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final double roll = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final CoordinateTransformation nedC = new CoordinateTransformation(
                        roll, pitch, yaw, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final Quaternion beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
                final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame);

                final BodyKinematics trueBeforeGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                                ecefFrame, ecefFrame);
                final BodyKinematics measuredBeforeGravityKinematics =
                        BodyKinematicsGenerator.generate(
                                TIME_INTERVAL_SECONDS,
                                trueBeforeGravityKinematics,
                                noErrorsInlier, random);

                final double beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final double beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final double beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final double deltaRoll = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));
                final double deltaPitch = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));
                final double deltaYaw = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));

                NEDFrame oldNedFrame = new NEDFrame(nedFrame);
                NEDFrame newNedFrame = new NEDFrame();
                ECEFFrame oldEcefFrame = new ECEFFrame();
                ECEFFrame newEcefFrame = new ECEFFrame();
                double oldRoll = roll - deltaRoll;
                double oldPitch = pitch - deltaPitch;
                double oldYaw = yaw - deltaYaw;

                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                        new BodyKinematicsSequence<>();
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        new BodyKinematicsSequence<>();
                sequence.setBeforeMeanSpecificForceCoordinates(
                        beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList =
                        new ArrayList<>();
                final List<StandardDeviationTimedBodyKinematics> measuredTimedKinematicsList =
                        new ArrayList<>();
                final boolean sequenceCanHaveOutliers =
                        randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                for (int j = 0; j < m; j++) {
                    final double newRoll = oldRoll + deltaRoll;
                    final double newPitch = oldPitch + deltaPitch;
                    final double newYaw = oldYaw + deltaYaw;
                    final CoordinateTransformation newNedC =
                            new CoordinateTransformation(
                                    newRoll, newPitch, newYaw,
                                    FrameType.BODY_FRAME,
                                    FrameType.LOCAL_NAVIGATION_FRAME);
                    final NEDPosition newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                            .estimateKinematicsAndReturnNew(
                                    TIME_INTERVAL_SECONDS, newEcefFrame,
                                    oldEcefFrame);

                    // apply known calibration parameters to distort ground-truth and generate a
                    // measured kinematics sample
                    final BodyKinematics measuredKinematics;
                    if (sequenceCanHaveOutliers &&
                            randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                        // outlier
                        measuredKinematics = BodyKinematicsGenerator
                                .generate(TIME_INTERVAL_SECONDS,
                                        trueKinematics, errorsOutlier,
                                        random);

                    } else {
                        // inlier
                        measuredKinematics = BodyKinematicsGenerator
                                .generate(TIME_INTERVAL_SECONDS,
                                        trueKinematics, errorsInlier,
                                        random);
                    }

                    final StandardDeviationTimedBodyKinematics trueTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(
                                    trueKinematics, timestampSeconds,
                                    specificForceStandardDeviation,
                                    angularRateStandardDeviation);

                    final StandardDeviationTimedBodyKinematics measuredTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(
                                    measuredKinematics, timestampSeconds,
                                    specificForceStandardDeviation,
                                    angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final Quaternion afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(
                        trueSequence, beforeQ, afterQ);

                final CoordinateTransformation newNedC =
                        new CoordinateTransformation(
                                afterQ.asInhomogeneousMatrix(),
                                FrameType.BODY_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final BodyKinematics trueAfterGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                                newEcefFrame, newEcefFrame);
                final BodyKinematics measuredAfterGravityKinematics =
                        BodyKinematicsGenerator.generate(
                                TIME_INTERVAL_SECONDS,
                                trueAfterGravityKinematics,
                                noErrorsInlier, random);

                final double afterMeanFx = measuredAfterGravityKinematics.getFx();
                final double afterMeanFy = measuredAfterGravityKinematics.getFy();
                final double afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(
                        afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final LMedSRobustEasyGyroscopeCalibrator calibrator =
                    new LMedSRobustEasyGyroscopeCalibrator(sequences,
                            true, false,
                            bg, mg, gg, ba, ma, this);
            final int subsetSize = calibrator.getMinimumRequiredMeasurementsOrSequences();
            calibrator.setPreliminarySubsetSize(subsetSize);
            calibrator.setStopThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);
            assertEquals(mCalibrateNextIteration, 0);
            assertEquals(mCalibrateProgressChange, 0);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!bg.equals(estimatedBg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

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
    public void testCalibrateGeneralAndGDependentCrossBiasesDisabledWithInlierNoise()
            throws WrongSizeException, InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateGeneralMg();
            final Matrix gg = new Matrix(3, 3);
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
            final IMUErrors noErrorsInlier = new IMUErrors(ba, bg, ma, mg, gg,
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
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final int m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final double roll = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final CoordinateTransformation nedC = new CoordinateTransformation(
                        roll, pitch, yaw, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final Quaternion beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
                final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame);

                final BodyKinematics trueBeforeGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                                ecefFrame, ecefFrame);
                final BodyKinematics measuredBeforeGravityKinematics =
                        BodyKinematicsGenerator.generate(
                                TIME_INTERVAL_SECONDS,
                                trueBeforeGravityKinematics,
                                noErrorsInlier, random);

                final double beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final double beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final double beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final double deltaRoll = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));
                final double deltaPitch = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));
                final double deltaYaw = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));

                NEDFrame oldNedFrame = new NEDFrame(nedFrame);
                NEDFrame newNedFrame = new NEDFrame();
                ECEFFrame oldEcefFrame = new ECEFFrame();
                ECEFFrame newEcefFrame = new ECEFFrame();
                double oldRoll = roll - deltaRoll;
                double oldPitch = pitch - deltaPitch;
                double oldYaw = yaw - deltaYaw;

                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                        new BodyKinematicsSequence<>();
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        new BodyKinematicsSequence<>();
                sequence.setBeforeMeanSpecificForceCoordinates(
                        beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList =
                        new ArrayList<>();
                final List<StandardDeviationTimedBodyKinematics> measuredTimedKinematicsList =
                        new ArrayList<>();
                final boolean sequenceCanHaveOutliers =
                        randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                for (int j = 0; j < m; j++) {
                    final double newRoll = oldRoll + deltaRoll;
                    final double newPitch = oldPitch + deltaPitch;
                    final double newYaw = oldYaw + deltaYaw;
                    final CoordinateTransformation newNedC =
                            new CoordinateTransformation(
                                    newRoll, newPitch, newYaw,
                                    FrameType.BODY_FRAME,
                                    FrameType.LOCAL_NAVIGATION_FRAME);
                    final NEDPosition newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                            .estimateKinematicsAndReturnNew(
                                    TIME_INTERVAL_SECONDS, newEcefFrame,
                                    oldEcefFrame);

                    // apply known calibration parameters to distort ground-truth and generate a
                    // measured kinematics sample
                    final BodyKinematics measuredKinematics;
                    if (sequenceCanHaveOutliers &&
                            randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                        // outlier
                        measuredKinematics = BodyKinematicsGenerator
                                .generate(TIME_INTERVAL_SECONDS,
                                        trueKinematics, errorsOutlier,
                                        random);

                    } else {
                        // inlier
                        measuredKinematics = BodyKinematicsGenerator
                                .generate(TIME_INTERVAL_SECONDS,
                                        trueKinematics, errorsInlier,
                                        random);
                    }

                    final StandardDeviationTimedBodyKinematics trueTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(
                                    trueKinematics, timestampSeconds,
                                    specificForceStandardDeviation,
                                    angularRateStandardDeviation);

                    final StandardDeviationTimedBodyKinematics measuredTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(
                                    measuredKinematics, timestampSeconds,
                                    specificForceStandardDeviation,
                                    angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final Quaternion afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(
                        trueSequence, beforeQ, afterQ);

                final CoordinateTransformation newNedC =
                        new CoordinateTransformation(
                                afterQ.asInhomogeneousMatrix(),
                                FrameType.BODY_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final BodyKinematics trueAfterGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                                newEcefFrame, newEcefFrame);
                final BodyKinematics measuredAfterGravityKinematics =
                        BodyKinematicsGenerator.generate(
                                TIME_INTERVAL_SECONDS,
                                trueAfterGravityKinematics,
                                noErrorsInlier, random);

                final double afterMeanFx = measuredAfterGravityKinematics.getFx();
                final double afterMeanFy = measuredAfterGravityKinematics.getFy();
                final double afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(
                        afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final LMedSRobustEasyGyroscopeCalibrator calibrator =
                    new LMedSRobustEasyGyroscopeCalibrator(sequences,
                            false, false,
                            bg, mg, gg, ba, ma, this);
            final int subsetSize = calibrator.getMinimumRequiredMeasurementsOrSequences();
            calibrator.setPreliminarySubsetSize(subsetSize);
            calibrator.setStopThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);
            assertEquals(mCalibrateNextIteration, 0);
            assertEquals(mCalibrateProgressChange, 0);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!bg.equals(estimatedBg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

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
    public void testCalibrateCommonAxisAndGDependentCrossBiasesEnabledWithInlierNoise()
            throws WrongSizeException, InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateCommonAxisMg();
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
            final IMUErrors noErrorsInlier = new IMUErrors(ba, bg, ma, mg, gg,
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
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final int m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final double roll = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final CoordinateTransformation nedC = new CoordinateTransformation(
                        roll, pitch, yaw, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final Quaternion beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
                final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame);

                final BodyKinematics trueBeforeGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                                ecefFrame, ecefFrame);
                final BodyKinematics measuredBeforeGravityKinematics =
                        BodyKinematicsGenerator.generate(
                                TIME_INTERVAL_SECONDS,
                                trueBeforeGravityKinematics,
                                noErrorsInlier, random);

                final double beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final double beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final double beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final double deltaRoll = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));
                final double deltaPitch = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));
                final double deltaYaw = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));

                NEDFrame oldNedFrame = new NEDFrame(nedFrame);
                NEDFrame newNedFrame = new NEDFrame();
                ECEFFrame oldEcefFrame = new ECEFFrame();
                ECEFFrame newEcefFrame = new ECEFFrame();
                double oldRoll = roll - deltaRoll;
                double oldPitch = pitch - deltaPitch;
                double oldYaw = yaw - deltaYaw;

                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                        new BodyKinematicsSequence<>();
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        new BodyKinematicsSequence<>();
                sequence.setBeforeMeanSpecificForceCoordinates(
                        beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList =
                        new ArrayList<>();
                final List<StandardDeviationTimedBodyKinematics> measuredTimedKinematicsList =
                        new ArrayList<>();
                final boolean sequenceCanHaveOutliers =
                        randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                for (int j = 0; j < m; j++) {
                    final double newRoll = oldRoll + deltaRoll;
                    final double newPitch = oldPitch + deltaPitch;
                    final double newYaw = oldYaw + deltaYaw;
                    final CoordinateTransformation newNedC =
                            new CoordinateTransformation(
                                    newRoll, newPitch, newYaw,
                                    FrameType.BODY_FRAME,
                                    FrameType.LOCAL_NAVIGATION_FRAME);
                    final NEDPosition newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                            .estimateKinematicsAndReturnNew(
                                    TIME_INTERVAL_SECONDS, newEcefFrame,
                                    oldEcefFrame);

                    // apply known calibration parameters to distort ground-truth and generate a
                    // measured kinematics sample
                    final BodyKinematics measuredKinematics;
                    if (sequenceCanHaveOutliers &&
                            randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                        // outlier
                        measuredKinematics = BodyKinematicsGenerator
                                .generate(TIME_INTERVAL_SECONDS,
                                        trueKinematics, errorsOutlier,
                                        random);

                    } else {
                        // inlier
                        measuredKinematics = BodyKinematicsGenerator
                                .generate(TIME_INTERVAL_SECONDS,
                                        trueKinematics, errorsInlier,
                                        random);
                    }

                    final StandardDeviationTimedBodyKinematics trueTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(
                                    trueKinematics, timestampSeconds,
                                    specificForceStandardDeviation,
                                    angularRateStandardDeviation);

                    final StandardDeviationTimedBodyKinematics measuredTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(
                                    measuredKinematics, timestampSeconds,
                                    specificForceStandardDeviation,
                                    angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final Quaternion afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(
                        trueSequence, beforeQ, afterQ);

                final CoordinateTransformation newNedC =
                        new CoordinateTransformation(
                                afterQ.asInhomogeneousMatrix(),
                                FrameType.BODY_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final BodyKinematics trueAfterGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                                newEcefFrame, newEcefFrame);
                final BodyKinematics measuredAfterGravityKinematics =
                        BodyKinematicsGenerator.generate(
                                TIME_INTERVAL_SECONDS,
                                trueAfterGravityKinematics,
                                noErrorsInlier, random);

                final double afterMeanFx = measuredAfterGravityKinematics.getFx();
                final double afterMeanFy = measuredAfterGravityKinematics.getFy();
                final double afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(
                        afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final LMedSRobustEasyGyroscopeCalibrator calibrator =
                    new LMedSRobustEasyGyroscopeCalibrator(sequences,
                            true, true,
                            bg, mg, gg, ba, ma, this);
            final int subsetSize = calibrator.getMinimumRequiredMeasurementsOrSequences();
            calibrator.setPreliminarySubsetSize(subsetSize);
            calibrator.setStopThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);
            assertEquals(mCalibrateNextIteration, 0);
            assertEquals(mCalibrateProgressChange, 0);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!bg.equals(estimatedBg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkCommonAxisAndGDependantCrossBiasesCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(calibrator.getEstimatedChiSq(), 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralAndGDependentCrossBiasesEnabledWithInlierNoise()
            throws WrongSizeException, InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix bg = generateBg();
            final Matrix ma = generateMa();
            final Matrix mg = generateGeneralMg();
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
            final IMUErrors noErrorsInlier = new IMUErrors(ba, bg, ma, mg, gg,
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
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            final int m = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> sequences = new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                // initial attitude of sequence
                final double roll = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final CoordinateTransformation nedC = new CoordinateTransformation(
                        roll, pitch, yaw, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final Quaternion beforeQ = new Quaternion();
                nedC.asRotation(beforeQ);

                final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
                final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame);

                final BodyKinematics trueBeforeGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                                ecefFrame, ecefFrame);
                final BodyKinematics measuredBeforeGravityKinematics =
                        BodyKinematicsGenerator.generate(
                                TIME_INTERVAL_SECONDS,
                                trueBeforeGravityKinematics,
                                noErrorsInlier, random);

                final double beforeMeanFx = measuredBeforeGravityKinematics.getFx();
                final double beforeMeanFy = measuredBeforeGravityKinematics.getFy();
                final double beforeMeanFz = measuredBeforeGravityKinematics.getFz();

                final double deltaRoll = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));
                final double deltaPitch = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));
                final double deltaYaw = Math.toRadians(
                        randomizer.nextDouble(
                                MIN_ANGLE_VARIATION_DEGREES,
                                MAX_ANGLE_VARIATION_DEGREES));

                NEDFrame oldNedFrame = new NEDFrame(nedFrame);
                NEDFrame newNedFrame = new NEDFrame();
                ECEFFrame oldEcefFrame = new ECEFFrame();
                ECEFFrame newEcefFrame = new ECEFFrame();
                double oldRoll = roll - deltaRoll;
                double oldPitch = pitch - deltaPitch;
                double oldYaw = yaw - deltaYaw;

                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                        new BodyKinematicsSequence<>();
                final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                        new BodyKinematicsSequence<>();
                sequence.setBeforeMeanSpecificForceCoordinates(
                        beforeMeanFx, beforeMeanFy, beforeMeanFz);

                final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList =
                        new ArrayList<>();
                final List<StandardDeviationTimedBodyKinematics> measuredTimedKinematicsList =
                        new ArrayList<>();
                final boolean sequenceCanHaveOutliers =
                        randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE;
                for (int j = 0; j < m; j++) {
                    final double newRoll = oldRoll + deltaRoll;
                    final double newPitch = oldPitch + deltaPitch;
                    final double newYaw = oldYaw + deltaYaw;
                    final CoordinateTransformation newNedC =
                            new CoordinateTransformation(
                                    newRoll, newPitch, newYaw,
                                    FrameType.BODY_FRAME,
                                    FrameType.LOCAL_NAVIGATION_FRAME);
                    final NEDPosition newNedPosition = oldNedFrame.getPosition();

                    newNedFrame.setPosition(newNedPosition);
                    newNedFrame.setCoordinateTransformation(newNedC);

                    NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);
                    NEDtoECEFFrameConverter.convertNEDtoECEF(oldNedFrame, oldEcefFrame);

                    final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

                    // compute ground-truth kinematics that should be generated at provided
                    // position, velocity and orientation
                    final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                            .estimateKinematicsAndReturnNew(
                                    TIME_INTERVAL_SECONDS, newEcefFrame,
                                    oldEcefFrame);

                    // apply known calibration parameters to distort ground-truth and generate a
                    // measured kinematics sample
                    final BodyKinematics measuredKinematics;
                    if (sequenceCanHaveOutliers &&
                            randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                        // outlier
                        measuredKinematics = BodyKinematicsGenerator
                                .generate(TIME_INTERVAL_SECONDS,
                                        trueKinematics, errorsOutlier,
                                        random);

                    } else {
                        // inlier
                        measuredKinematics = BodyKinematicsGenerator
                                .generate(TIME_INTERVAL_SECONDS,
                                        trueKinematics, errorsInlier,
                                        random);
                    }

                    final StandardDeviationTimedBodyKinematics trueTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(
                                    trueKinematics, timestampSeconds,
                                    specificForceStandardDeviation,
                                    angularRateStandardDeviation);

                    final StandardDeviationTimedBodyKinematics measuredTimedKinematics =
                            new StandardDeviationTimedBodyKinematics(
                                    measuredKinematics, timestampSeconds,
                                    specificForceStandardDeviation,
                                    angularRateStandardDeviation);

                    trueTimedKinematicsList.add(trueTimedKinematics);
                    measuredTimedKinematicsList.add(measuredTimedKinematics);

                    oldNedFrame.copyFrom(newNedFrame);
                    oldRoll = newRoll;
                    oldPitch = newPitch;
                    oldYaw = newYaw;
                }
                trueSequence.setItems(trueTimedKinematicsList);
                sequence.setItems(measuredTimedKinematicsList);

                final Quaternion afterQ = new Quaternion();
                QuaternionIntegrator.integrateGyroSequence(
                        trueSequence, beforeQ, afterQ);

                final CoordinateTransformation newNedC =
                        new CoordinateTransformation(
                                afterQ.asInhomogeneousMatrix(),
                                FrameType.BODY_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME);

                newNedFrame.setPosition(nedPosition);
                newNedFrame.setCoordinateTransformation(newNedC);

                NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

                final BodyKinematics trueAfterGravityKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                                newEcefFrame, newEcefFrame);
                final BodyKinematics measuredAfterGravityKinematics =
                        BodyKinematicsGenerator.generate(
                                TIME_INTERVAL_SECONDS,
                                trueAfterGravityKinematics,
                                noErrorsInlier, random);

                final double afterMeanFx = measuredAfterGravityKinematics.getFx();
                final double afterMeanFy = measuredAfterGravityKinematics.getFy();
                final double afterMeanFz = measuredAfterGravityKinematics.getFz();

                sequence.setAfterMeanSpecificForceCoordinates(
                        afterMeanFx, afterMeanFy, afterMeanFz);

                sequences.add(sequence);
            }

            final LMedSRobustEasyGyroscopeCalibrator calibrator =
                    new LMedSRobustEasyGyroscopeCalibrator(sequences,
                            false, true,
                            bg, mg, gg, ba, ma, this);
            final int subsetSize = calibrator.getMinimumRequiredMeasurementsOrSequences();
            calibrator.setPreliminarySubsetSize(subsetSize);
            calibrator.setStopThreshold(THRESHOLD);

            // estimate
            reset();
            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 0);
            assertEquals(mCalibrateEnd, 0);
            assertEquals(mCalibrateNextIteration, 0);
            assertEquals(mCalibrateProgressChange, 0);

            try {
                calibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            assertTrue(calibrator.isReady());
            assertFalse(calibrator.isRunning());
            assertEquals(mCalibrateStart, 1);
            assertEquals(mCalibrateEnd, 1);
            assertTrue(mCalibrateNextIteration > 0);
            assertTrue(mCalibrateProgressChange >= 0);

            final Matrix estimatedBg = calibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!bg.equals(estimatedBg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedBg, estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());
            checkGeneralAndGDependantCrossBiasesCovariance(calibrator.getEstimatedCovariance());
            assertTrue(calibrator.getEstimatedMse() > 0.0);
            assertNotEquals(calibrator.getEstimatedChiSq(), 0.0);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(
            final RobustEasyGyroscopeCalibrator calibrator) {
        checkLocked((LMedSRobustEasyGyroscopeCalibrator) calibrator);
        mCalibrateStart++;
    }

    @Override
    public void onCalibrateEnd(
            final RobustEasyGyroscopeCalibrator calibrator) {
        checkLocked((LMedSRobustEasyGyroscopeCalibrator) calibrator);
        mCalibrateEnd++;
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustEasyGyroscopeCalibrator calibrator,
            final int iteration) {
        checkLocked((LMedSRobustEasyGyroscopeCalibrator) calibrator);
        mCalibrateNextIteration++;
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustEasyGyroscopeCalibrator calibrator,
            final float progress) {
        checkLocked((LMedSRobustEasyGyroscopeCalibrator) calibrator);
        mCalibrateProgressChange++;
    }

    private void reset() {
        mCalibrateStart = 0;
        mCalibrateEnd = 0;
        mCalibrateNextIteration = 0;
        mCalibrateProgressChange = 0;
    }

    private void checkLocked(
            final LMedSRobustEasyGyroscopeCalibrator calibrator) {
        assertTrue(calibrator.isRunning());
        try {
            calibrator.setAccelerometerBiasX(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerBiasY(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerBiasZ(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerBiasX(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerBiasY(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerBiasZ(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerBias(
                    0.0, 0.0,
                    0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerBias(
                    null, null,
                    null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerBias((double[]) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerBias((Matrix) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerSx(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerSy(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerSz(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerMxy(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerMxz(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerMyx(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerMyz(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerMzx(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerMzy(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerScalingFactors(
                    0.0, 0.0,
                    0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerCrossCouplingErrors(
                    0.0, 0.0,
                    0.0, 0.0,
                    0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerScalingFactorsAndCrossCouplingErrors(
                    0.0, 0.0,
                    0.0, 0.0,
                    0.0, 0.0,
                    0.0, 0.0,
                    0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerMa(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
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
            calibrator.setInitialBias(0.0, 0.0,
                    0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setInitialBias(null, null,
                    null);
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
            calibrator.setSequences(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setCommonAxisUsed(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setGDependentCrossBiasesEstimated(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setProgressDelta(0.0f);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setConfidence(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setMaxIterations(1);
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
            calibrator.setPreliminarySubsetSize(1);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setStopThreshold(0.5);
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
            final RobustEasyGyroscopeCalibrator calibrator)
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
            final RobustEasyGyroscopeCalibrator calibrator) {
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

    private void checkCommonAxisAndGDependantCrossBiasesCovariance(
            final Matrix covariance) {
        assertEquals(covariance.getRows(), 21);
        assertEquals(covariance.getColumns(), 21);

        for (int j = 0; j < 21; j++) {
            final boolean colIsZero = j == 8 || j == 10 || j == 11;
            for (int i = 0; i < 21; i++) {
                final boolean rowIsZero = i == 8 || i == 10 || i == 11;
                if (colIsZero || rowIsZero) {
                    assertEquals(covariance.getElementAt(i, j), 0.0, 0.0);
                }
            }
        }
    }

    private void checkGeneralAndGDependantCrossBiasesCovariance(
            final Matrix covariance) {
        assertEquals(covariance.getRows(), 21);
        assertEquals(covariance.getColumns(), 21);

        for (int i = 0; i < 21; i++) {
            assertNotEquals(covariance.getElementAt(i, i), 0.0);
        }
    }

    private void checkCommonAxisCovariance(final Matrix covariance) {
        assertEquals(covariance.getRows(), 21);
        assertEquals(covariance.getColumns(), 21);

        for (int j = 0; j < 21; j++) {
            final boolean colIsZero = j == 8 || j > 9;
            for (int i = 0; i < 21; i++) {
                final boolean rowIsZero = i == 8 || i > 9;
                if (colIsZero || rowIsZero) {
                    assertEquals(covariance.getElementAt(i, j), 0.0, 0.0);
                }
            }
        }
    }

    private void checkGeneralCovariance(final Matrix covariance) {
        assertEquals(covariance.getRows(), 21);
        assertEquals(covariance.getColumns(), 21);

        for (int j = 0; j < 21; j++) {
            final boolean colIsZero = j > 11;
            for (int i = 0; i < 21; i++) {
                final boolean rowIsZero = i > 11;
                if (colIsZero || rowIsZero) {
                    assertEquals(covariance.getElementAt(i, j), 0.0, 0.0);
                }
            }
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

    private Matrix generateMa() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                -150e-6, -600e-6, 250e-6,
                -250e-6, 100e-6, 450e-6
        }, false);

        return result;
    }

    private Matrix generateCommonAxisMg() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                400e-6, -300e-6, 250e-6,
                0.0, -300e-6, -150e-6,
                0.0, 0.0, -350e-6
        }, false);

        return result;
    }

    private Matrix generateGeneralMg() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                400e-6, -300e-6, 250e-6,
                -300e-6, -300e-6, -150e-6,
                250e-6, -150e-6, -350e-6
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
