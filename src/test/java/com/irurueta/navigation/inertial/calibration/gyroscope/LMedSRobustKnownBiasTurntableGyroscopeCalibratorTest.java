package com.irurueta.navigation.inertial.calibration.gyroscope;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.AxisRotation3D;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.geometry.RotationException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.numerical.robust.RobustEstimatorMethod;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Assert;
import org.junit.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;

public class LMedSRobustKnownBiasTurntableGyroscopeCalibratorTest implements
        RobustKnownBiasTurntableGyroscopeCalibratorListener {

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

    private static final double MIN_ROTATION_RATE_DEGREES_PER_SECOND = 90.0;
    private static final double MAX_ROTATION_RATE_DEGREES_PER_SECOND = 180.0;

    private static final double MIN_TIME_INTERVAL = 0.01;
    private static final double MAX_TIME_INTERVAL = 0.1;

    private static final int MEASUREMENT_NUMBER = 1000;

    private static final int OUTLIER_PERCENTAGE = 5;

    private static final double THRESHOLD = 2e-1;
    private static final double LARGE_THRESHOLD = 5e-1;

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
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, new double[3], 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
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
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueY(), 0.0, 0.0);
        assertEquals(biasTriad1.getValueZ(), 0.0, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, new double[3], 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, new Matrix(3, 1));
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(),
                Constants.EARTH_ROTATION_RATE, 0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                Constants.EARTH_ROTATION_RATE, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        Assert.assertEquals(calibrator.getTimeInterval(),
                TurntableGyroscopeCalibrator.DEFAULT_TIME_INTERVAL,
                0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(),
                TurntableGyroscopeCalibrator.DEFAULT_TIME_INTERVAL,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertNull(calibrator.getMeasurements());
        assertNull(calibrator.getEcefPosition());
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 16);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        Assert.assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);
    }

    @Test
    public void testConstructor2() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                        timeInterval, measurements, bg, mg, gg);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, new double[3], 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
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
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 16);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    0.0, timeInterval, measurements,
                    bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, 0.0, measurements, bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    new Matrix(1, 1), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    new Matrix(3, 3), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg,
                    new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg,
                    new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg, mg,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg, mg,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor3() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                        rotationRate, timeInterval, measurements,
                        bg, mg, gg, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, new double[3], 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
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
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 16);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    0.0, timeInterval, measurements,
                    bg, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, 0.0, measurements, bg, mg, gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    new Matrix(1, 1), mg, gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    new Matrix(3, 3), mg, gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg,
                    new Matrix(1, 3), gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg,
                    new Matrix(3, 1), gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg, mg,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg, mg,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor4() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                        rotationRate, timeInterval, measurements,
                        bias, mg, gg);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, new double[3], 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
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
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 16);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    0.0, timeInterval, measurements,
                    bias, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, 0.0, measurements, bias, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    new double[1], mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias,
                    new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias,
                    new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias, mg,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias, mg,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor5() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                        rotationRate, timeInterval, measurements,
                        bias, mg, gg, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, new double[3], 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
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
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 16);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    0.0, timeInterval, measurements,
                    bias, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, 0.0, measurements, bias, mg, gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    new double[1], mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias,
                    new Matrix(1, 3), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias,
                    new Matrix(3, 1), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias, mg,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias, mg,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor6() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                        timeInterval, measurements, bias, mg, gg,
                        accelerometerBias, ma);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(calibrator.getAccelerometerSx(), asx, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), asy, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), asz, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), amxy, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), amxz, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), amyx, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), amyz, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), amzx, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), amzy, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 16);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    0.0, timeInterval, measurements,
                    bias, mg, gg, accelerometerBias, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, 0.0, measurements, bias, mg, gg,
                    accelerometerBias, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    new double[1], mg, gg, accelerometerBias, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias,
                    new Matrix(1, 3), gg, accelerometerBias,
                    ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias,
                    new Matrix(3, 1), gg, accelerometerBias,
                    ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias, mg,
                    new Matrix(1, 3), accelerometerBias,
                    ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias, mg,
                    new Matrix(3, 1), accelerometerBias,
                    ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias,
                    mg, gg, new double[1], ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias, mg, gg,
                    accelerometerBias, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias, mg, gg,
                    accelerometerBias, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor7() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                        timeInterval, measurements, bias, mg, gg,
                        accelerometerBias, ma, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(calibrator.getAccelerometerSx(), asx, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), asy, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), asz, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), amxy, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), amxz, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), amyx, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), amyz, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), amzx, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), amzy, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 16);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    0.0, timeInterval, measurements,
                    bias, mg, gg, accelerometerBias, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, 0.0, measurements, bias, mg, gg,
                    accelerometerBias, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    new double[1], mg, gg, accelerometerBias, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias,
                    new Matrix(1, 3), gg, accelerometerBias,
                    ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias,
                    new Matrix(3, 1), gg, accelerometerBias,
                    ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias, mg,
                    new Matrix(1, 3), accelerometerBias,
                    ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias, mg,
                    new Matrix(3, 1), accelerometerBias,
                    ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias,
                    mg, gg, new double[1], ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias, mg, gg,
                    accelerometerBias, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias, mg, gg,
                    accelerometerBias, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor8() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                        timeInterval, measurements, bg, mg, gg,
                        ba, ma);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(calibrator.getAccelerometerSx(), asx, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), asy, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), asz, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), amxy, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), amxz, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), amyx, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), amyz, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), amzx, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), amzy, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 16);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    0.0, timeInterval, measurements,
                    bg, mg, gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, 0.0, measurements, bg, mg, gg,
                    ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    new Matrix(1, 1), mg, gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    new Matrix(3, 3), mg, gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg,
                    new Matrix(1, 3), gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg,
                    new Matrix(3, 1), gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg, mg,
                    new Matrix(1, 3), ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg, mg,
                    new Matrix(3, 1), ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg,
                    mg, gg, new Matrix(1, 1), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg,
                    mg, gg, new Matrix(3, 3), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg, mg, gg,
                    ba, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg, mg, gg,
                    ba, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor9() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                        timeInterval, measurements, bg, mg, gg, ba, ma,
                        this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(calibrator.getAccelerometerSx(), asx, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), asy, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), asz, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), amxy, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), amxz, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), amyx, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), amyz, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), amzx, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), amzy, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 16);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    0.0, timeInterval, measurements,
                    bg, mg, gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, 0.0, measurements, bg, mg, gg,
                    ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    new Matrix(1, 1), mg, gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    new Matrix(3, 3), mg, gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg,
                    new Matrix(1, 3), gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg,
                    new Matrix(3, 1), gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg, mg,
                    new Matrix(1, 3), ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg, mg,
                    new Matrix(3, 1), ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg,
                    mg, gg, new Matrix(1, 1), ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg,
                    mg, gg, new Matrix(3, 3), ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg, mg, gg,
                    ba, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bg, mg, gg,
                    ba, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor10() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                        timeInterval, measurements, false, false, bg, mg, gg);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, new double[3], 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
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
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    0.0, timeInterval, measurements,
                    true, false, bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, 0.0, measurements,
                    true,
                    false, bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new Matrix(1, 1), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new Matrix(3, 3), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false, bg,
                    new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor11() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                        timeInterval, measurements, false,
                        false, bg, mg, gg, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, new double[3], 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
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
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    0.0, timeInterval, measurements,
                    true, false, bg, mg, gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, 0.0, measurements,
                    true,
                    false, bg, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new Matrix(1, 1), mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new Matrix(3, 3), mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false, bg,
                    new Matrix(1, 3), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, new Matrix(3, 1), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor12() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                        timeInterval, measurements, false,
                        false, bias, mg, gg);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, new double[3], 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
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
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    0.0, timeInterval, measurements,
                    true, false, bias, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, 0.0, measurements,
                    true,
                    false, bias, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new double[1], mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false, bias,
                    new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor13() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                        timeInterval, measurements, false,
                        false, bias, mg, gg, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, new double[3], 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
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
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    0.0, timeInterval, measurements,
                    true, false, bias, mg, gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, 0.0, measurements,
                    true,
                    false, bias, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new double[1], mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false, bias,
                    new Matrix(1, 3), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, new Matrix(3, 1), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor14() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                        timeInterval, measurements, false,
                        false, bias, mg, gg,
                        accelerometerBias, ma);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(calibrator.getAccelerometerSx(), asx, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), asy, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), asz, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), amxy, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), amxz, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), amyx, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), amyz, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), amzx, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), amzy, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    0.0, timeInterval, measurements,
                    true, false, bias, mg, gg,
                    accelerometerBias, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, 0.0, measurements,
                    true,
                    false, bias, mg, gg,
                    accelerometerBias, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new double[1], mg, gg, accelerometerBias, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false, bias,
                    new Matrix(1, 3), gg, accelerometerBias, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, new Matrix(3, 1), gg, accelerometerBias, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, new Matrix(1, 3), accelerometerBias, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, new Matrix(3, 1), accelerometerBias, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, gg, new double[1], ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, gg, accelerometerBias,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, gg, accelerometerBias,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor15() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                        timeInterval, measurements, false,
                        false, bias, mg, gg,
                        accelerometerBias, ma, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(calibrator.getAccelerometerSx(), asx, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), asy, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), asz, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), amxy, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), amxz, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), amyx, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), amyz, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), amzx, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), amzy, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    0.0, timeInterval, measurements,
                    true, false, bias, mg, gg,
                    accelerometerBias, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, 0.0, measurements,
                    true,
                    false, bias, mg, gg,
                    accelerometerBias, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new double[1], mg, gg, accelerometerBias, ma,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false, bias,
                    new Matrix(1, 3), gg, accelerometerBias, ma,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, new Matrix(3, 1), gg, accelerometerBias, ma,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, new Matrix(1, 3), accelerometerBias, ma,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, new Matrix(3, 1), accelerometerBias, ma,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, gg, new double[1], ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, gg, accelerometerBias,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, gg, accelerometerBias,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor16() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                        timeInterval, measurements, false,
                        false, bg, mg, gg,
                        ba, ma);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(calibrator.getAccelerometerSx(), asx, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), asy, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), asz, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), amxy, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), amxz, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), amyx, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), amyz, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), amzx, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), amzy, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    0.0, timeInterval, measurements,
                    true, false, bg, mg, gg,
                    ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, 0.0, measurements,
                    true,
                    false, bg, mg, gg,
                    ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new Matrix(1, 1), mg, gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new Matrix(3, 3), mg, gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false, bg,
                    new Matrix(1, 3), gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, new Matrix(3, 1), gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, new Matrix(1, 3), ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, new Matrix(3, 1), ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, gg, new Matrix(1, 1), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, gg, new Matrix(3, 3), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, gg, ba,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, gg, ba,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor17() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition, rotationRate,
                        timeInterval, measurements, false,
                        false, bg, mg, gg,
                        ba, ma, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(calibrator.getAccelerometerSx(), asx, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), asy, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), asz, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), amxy, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), amxz, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), amyx, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), amyz, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), amzx, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), amzy, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertSame(calibrator.getEcefPosition(), ecefPosition);
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    0.0, timeInterval, measurements,
                    true, false, bg, mg, gg,
                    ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, 0.0, measurements,
                    true,
                    false, bg, mg, gg,
                    ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new Matrix(1, 1), mg, gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new Matrix(3, 3), mg, gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false, bg,
                    new Matrix(1, 3), gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, new Matrix(3, 1), gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, new Matrix(1, 3), ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, new Matrix(3, 1), ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, gg, new Matrix(1, 1), ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, gg, new Matrix(3, 3), ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, gg, ba,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, gg, ba,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor18() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate,
                        timeInterval, measurements, bg, mg, gg);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, new double[3], 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
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
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 16);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    0.0, timeInterval, measurements,
                    bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, 0.0, measurements, bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    new Matrix(1, 1), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    new Matrix(3, 3), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg,
                    new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg,
                    new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg, mg,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg, mg,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor19() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                        rotationRate, timeInterval, measurements,
                        bg, mg, gg, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, new double[3], 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
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
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 16);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    0.0, timeInterval, measurements,
                    bg, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, 0.0, measurements, bg, mg, gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    new Matrix(1, 1), mg, gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    new Matrix(3, 3), mg, gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg,
                    new Matrix(1, 3), gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg,
                    new Matrix(3, 1), gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg, mg,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg, mg,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor20() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                        rotationRate, timeInterval, measurements,
                        bias, mg, gg);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, new double[3], 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
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
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 16);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    0.0, timeInterval, measurements,
                    bias, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, 0.0, measurements, bias, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    new double[1], mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias,
                    new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias,
                    new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias, mg,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias, mg,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor21() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                        rotationRate, timeInterval, measurements,
                        bias, mg, gg, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, new double[3], 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
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
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 16);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    0.0, timeInterval, measurements,
                    bias, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, 0.0, measurements, bias, mg, gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    new double[1], mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias,
                    new Matrix(1, 3), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias,
                    new Matrix(3, 1), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias, mg,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias, mg,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor22() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate,
                        timeInterval, measurements, bias, mg, gg,
                        accelerometerBias, ma);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(calibrator.getAccelerometerSx(), asx, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), asy, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), asz, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), amxy, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), amxz, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), amyx, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), amyz, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), amzx, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), amzy, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 16);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    0.0, timeInterval, measurements,
                    bias, mg, gg, accelerometerBias, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, 0.0, measurements, bias, mg, gg,
                    accelerometerBias, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    new double[1], mg, gg, accelerometerBias, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias,
                    new Matrix(1, 3), gg, accelerometerBias,
                    ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias,
                    new Matrix(3, 1), gg, accelerometerBias,
                    ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias, mg,
                    new Matrix(1, 3), accelerometerBias,
                    ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias, mg,
                    new Matrix(3, 1), accelerometerBias,
                    ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(ecefPosition,
                    rotationRate, timeInterval, measurements, bias,
                    mg, gg, new double[1], ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias, mg, gg,
                    accelerometerBias, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias, mg, gg,
                    accelerometerBias, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor23() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate,
                        timeInterval, measurements, bias, mg, gg,
                        accelerometerBias, ma, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(calibrator.getAccelerometerSx(), asx, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), asy, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), asz, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), amxy, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), amxz, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), amyx, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), amyz, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), amzx, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), amzy, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 16);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    0.0, timeInterval, measurements,
                    bias, mg, gg, accelerometerBias, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, 0.0, measurements, bias, mg, gg,
                    accelerometerBias, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    new double[1], mg, gg, accelerometerBias, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias,
                    new Matrix(1, 3), gg, accelerometerBias,
                    ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias,
                    new Matrix(3, 1), gg, accelerometerBias,
                    ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias, mg,
                    new Matrix(1, 3), accelerometerBias,
                    ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias, mg,
                    new Matrix(3, 1), accelerometerBias,
                    ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias,
                    mg, gg, new double[1], ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias, mg, gg,
                    accelerometerBias, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bias, mg, gg,
                    accelerometerBias, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor24() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate,
                        timeInterval, measurements, bg, mg, gg,
                        ba, ma);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(calibrator.getAccelerometerSx(), asx, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), asy, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), asz, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), amxy, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), amxz, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), amyx, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), amyz, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), amzx, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), amzy, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 16);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    0.0, timeInterval, measurements,
                    bg, mg, gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, 0.0, measurements, bg, mg, gg,
                    ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    new Matrix(1, 1), mg, gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    new Matrix(3, 3), mg, gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg,
                    new Matrix(1, 3), gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg,
                    new Matrix(3, 1), gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg, mg,
                    new Matrix(1, 3), ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg, mg,
                    new Matrix(3, 1), ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg,
                    mg, gg, new Matrix(1, 1), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg,
                    mg, gg, new Matrix(3, 3), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg, mg, gg,
                    ba, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg, mg, gg,
                    ba, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor25() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate,
                        timeInterval, measurements, bg, mg, gg, ba, ma,
                        this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(calibrator.getAccelerometerSx(), asx, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), asy, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), asz, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), amxy, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), amxz, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), amyx, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), amyz, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), amzx, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), amzy, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 16);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    0.0, timeInterval, measurements,
                    bg, mg, gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, 0.0, measurements, bg, mg, gg,
                    ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    new Matrix(1, 1), mg, gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    new Matrix(3, 3), mg, gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg,
                    new Matrix(1, 3), gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg,
                    new Matrix(3, 1), gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg, mg,
                    new Matrix(1, 3), ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg, mg,
                    new Matrix(3, 1), ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg,
                    mg, gg, new Matrix(1, 1), ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg,
                    mg, gg, new Matrix(3, 3), ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg, mg, gg,
                    ba, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements, bg, mg, gg,
                    ba, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor26() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate,
                        timeInterval, measurements, false,
                        false, bg, mg, gg);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, new double[3], 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
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
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    0.0, timeInterval, measurements,
                    true, false, bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, 0.0, measurements,
                    true,
                    false, bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new Matrix(1, 1), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new Matrix(3, 3), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false, bg,
                    new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor27() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate,
                        timeInterval, measurements, false,
                        false, bg, mg, gg, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, new double[3], 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
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
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    0.0, timeInterval, measurements,
                    true, false, bg, mg, gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, 0.0, measurements,
                    true,
                    false, bg, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new Matrix(1, 1), mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new Matrix(3, 3), mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false, bg,
                    new Matrix(1, 3), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, new Matrix(3, 1), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor28() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate,
                        timeInterval, measurements, false,
                        false, bias, mg, gg);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, new double[3], 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
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
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    0.0, timeInterval, measurements,
                    true, false, bias, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, 0.0, measurements,
                    true,
                    false, bias, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new double[1], mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false, bias,
                    new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor29() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate,
                        timeInterval, measurements, false,
                        false, bias, mg, gg, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, new double[3], 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, new Matrix(3, 1));
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
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
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    0.0, timeInterval, measurements,
                    true, false, bias, mg, gg,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, 0.0, measurements,
                    true,
                    false, bias, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new double[1], mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false, bias,
                    new Matrix(1, 3), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, new Matrix(3, 1), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor30() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate,
                        timeInterval, measurements, false,
                        false, bias, mg, gg,
                        accelerometerBias, ma);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(calibrator.getAccelerometerSx(), asx, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), asy, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), asz, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), amxy, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), amxz, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), amyx, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), amyz, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), amzx, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), amzy, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    0.0, timeInterval, measurements,
                    true, false, bias, mg, gg,
                    accelerometerBias, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, 0.0, measurements,
                    true,
                    false, bias, mg, gg,
                    accelerometerBias, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new double[1], mg, gg, accelerometerBias, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false, bias,
                    new Matrix(1, 3), gg, accelerometerBias, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, new Matrix(3, 1), gg, accelerometerBias, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, new Matrix(1, 3), accelerometerBias, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, new Matrix(3, 1), accelerometerBias, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, gg, new double[1], ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, gg, accelerometerBias,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, gg, accelerometerBias,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor31() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate,
                        timeInterval, measurements, false,
                        false, bias, mg, gg,
                        accelerometerBias, ma, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(calibrator.getAccelerometerSx(), asx, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), asy, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), asz, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), amxy, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), amxz, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), amyx, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), amyz, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), amzx, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), amzy, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    0.0, timeInterval, measurements,
                    true, false, bias, mg, gg,
                    accelerometerBias, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, 0.0, measurements,
                    true,
                    false, bias, mg, gg,
                    accelerometerBias, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new double[1], mg, gg, accelerometerBias, ma,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false, bias,
                    new Matrix(1, 3), gg, accelerometerBias, ma,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, new Matrix(3, 1), gg, accelerometerBias, ma,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, new Matrix(1, 3), accelerometerBias, ma,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, new Matrix(3, 1), accelerometerBias, ma,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, gg, new double[1], ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, gg, accelerometerBias,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bias, mg, gg, accelerometerBias,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor32() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate,
                        timeInterval, measurements, false,
                        false, bg, mg, gg,
                        ba, ma);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(calibrator.getAccelerometerSx(), asx, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), asy, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), asz, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), amxy, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), amxz, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), amyx, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), amyz, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), amzx, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), amzy, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertNull(calibrator.getListener());
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    0.0, timeInterval, measurements,
                    true, false, bg, mg, gg,
                    ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, 0.0, measurements,
                    true,
                    false, bg, mg, gg,
                    ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new Matrix(1, 1), mg, gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new Matrix(3, 3), mg, gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false, bg,
                    new Matrix(1, 3), gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, new Matrix(3, 1), gg, ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, new Matrix(1, 3), ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, new Matrix(3, 1), ba, ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, gg, new Matrix(1, 1), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, gg, new Matrix(3, 3), ma);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, gg, ba,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, gg, ba,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testConstructor33() throws WrongSizeException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);
        final NEDVelocity nedVelocity = new NEDVelocity();
        final ECEFPosition ecefPosition = new ECEFPosition();
        final ECEFVelocity ecefVelocity = new ECEFVelocity();
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(nedPosition, nedVelocity,
                ecefPosition, ecefVelocity);

        final double rotationRate = getTurntableRotationRate();
        final double timeInterval = randomizer.nextDouble(
                MIN_TIME_INTERVAL, MAX_TIME_INTERVAL);
        final List<StandardDeviationBodyKinematics> measurements =
                Collections.emptyList();
        final Matrix bg = generateBg();
        final Matrix mg = generateCommonAxisMg();
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
        final double[] accelerometerBias = ba.getBuffer();

        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition, rotationRate,
                        timeInterval, measurements, false,
                        false, bg, mg, gg,
                        ba, ma, this);

        // check default values
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);
        assertEquals(calibrator.getMethod(), RobustEstimatorMethod.LMedS);
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
        final Acceleration accelerationX1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(accelerationX1.getValue().doubleValue(), bax, 0.0);
        assertEquals(accelerationX1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationX2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(accelerationX2);
        assertEquals(accelerationX1, accelerationX2);
        final Acceleration accelerationY1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(accelerationY1.getValue().doubleValue(), bay, 0.0);
        assertEquals(accelerationY1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(accelerationY2);
        assertEquals(accelerationY1, accelerationY2);
        final Acceleration accelerationZ1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(accelerationZ1.getValue().doubleValue(), baz, 0.0);
        assertEquals(accelerationZ1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ2 = new Acceleration(
                0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(accelerationZ2);
        assertEquals(accelerationZ1, accelerationZ2);
        final double[] accelerometerBias1 = calibrator.getAccelerometerBias();
        assertArrayEquals(accelerometerBias1, accelerometerBias, 0.0);
        final double[] accelerometerBias2 = new double[3];
        calibrator.getAccelerometerBias(accelerometerBias2);
        assertArrayEquals(accelerometerBias1, accelerometerBias2, 0.0);
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        assertEquals(ba1, ba);
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);
        assertEquals(calibrator.getAccelerometerSx(), asx, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), asy, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), asz, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), amxy, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), amxz, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), amyx, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), amyz, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), amzx, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), amzy, 0.0);
        final Matrix ma1 = calibrator.getAccelerometerMa();
        assertEquals(ma1, ma);
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);
        assertEquals(ma1, ma2);
        assertEquals(calibrator.getBiasX(), bgx, 0.0);
        assertEquals(calibrator.getBiasY(), bgy, 0.0);
        assertEquals(calibrator.getBiasZ(), bgz, 0.0);
        final AngularSpeed angularSpeedX1 = calibrator.getBiasAngularSpeedX();
        assertEquals(angularSpeedX1.getValue().doubleValue(), bgx, 0.0);
        assertEquals(angularSpeedX1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedX2 = new AngularSpeed(
                0.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeedX2);
        assertEquals(angularSpeedX1, angularSpeedX2);
        final AngularSpeed angularSpeedY1 = calibrator.getBiasAngularSpeedY();
        assertEquals(angularSpeedY1.getValue().doubleValue(), bgy, 0.0);
        assertEquals(angularSpeedY1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeedY2);
        assertEquals(angularSpeedY1, angularSpeedY2);
        final AngularSpeed angularSpeedZ1 = calibrator.getBiasAngularSpeedZ();
        assertEquals(angularSpeedZ1.getValue().doubleValue(), bgz, 0.0);
        assertEquals(angularSpeedZ1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ2 = new AngularSpeed(
                0.0, AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeedZ2);
        assertEquals(angularSpeedZ1, angularSpeedZ2);
        final AngularSpeedTriad biasTriad1 = calibrator.getBiasAsTriad();
        assertEquals(biasTriad1.getValueX(), bgx, 0.0);
        assertEquals(biasTriad1.getValueY(), bgy, 0.0);
        assertEquals(biasTriad1.getValueZ(), bgz, 0.0);
        assertEquals(biasTriad1.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeedTriad biasTriad2 = new AngularSpeedTriad();
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
        final double[] bias1 = calibrator.getBias();
        assertArrayEquals(bias1, bias, 0.0);
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);
        assertArrayEquals(bias1, bias2, 0.0);
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        assertEquals(bg1, bg);
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);
        assertEquals(bg1, bg2);
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
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);
        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                rotationRate, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed rotationRate2 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotationRate2);
        assertEquals(rotationRate1, rotationRate2);
        assertEquals(calibrator.getTimeInterval(), timeInterval, 0.0);
        final Time time1 = calibrator.getTimeIntervalAsTime();
        assertEquals(time1.getValue().doubleValue(), timeInterval,
                0.0);
        assertEquals(time1.getUnit(), TimeUnit.SECOND);
        final Time time2 = new Time(0.0, TimeUnit.MILLISECOND);
        calibrator.getTimeIntervalAsTime(time2);
        assertEquals(time1, time2);
        assertSame(calibrator.getMeasurements(), measurements);
        assertTrue(calibrator.getEcefPosition().equals(ecefPosition, LARGE_ABSOLUTE_ERROR));
        assertTrue(calibrator.getNedPosition().equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(nedPosition2));
        assertTrue(nedPosition2.equals(nedPosition, LARGE_ABSOLUTE_ERROR));
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
        assertSame(calibrator.getListener(), this);
        assertEquals(calibrator.getMinimumRequiredMeasurements(), 10);
        assertFalse(calibrator.isReady());
        assertFalse(calibrator.isRunning());
        assertEquals(calibrator.getProgressDelta(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);
        assertEquals(calibrator.getConfidence(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);
        assertEquals(calibrator.getMaxIterations(),
                RobustTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);
        assertNull(calibrator.getInliersData());
        assertTrue(calibrator.isResultRefined());
        assertTrue(calibrator.isCovarianceKept());
        assertNull(calibrator.getQualityScores());
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
        assertEquals(calibrator.getPreliminarySubsetSize(), TurntableGyroscopeCalibrator
                .MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

        // Force IllegalArgumentException
        calibrator = null;
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    0.0, timeInterval, measurements,
                    true, false, bg, mg, gg,
                    ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, 0.0, measurements,
                    true,
                    false, bg, mg, gg,
                    ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new Matrix(1, 1), mg, gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    new Matrix(3, 3), mg, gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false, bg,
                    new Matrix(1, 3), gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, new Matrix(3, 1), gg, ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, new Matrix(1, 3), ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, new Matrix(3, 1), ba, ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, gg, new Matrix(1, 1), ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, gg, new Matrix(3, 3), ma, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, gg, ba,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator = new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                    rotationRate, timeInterval, measurements,
                    true,
                    false,
                    bg, mg, gg, ba,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(calibrator);
    }

    @Test
    public void testGetSetStopThreshold() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getStopThreshold(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_STOP_THRESHOLD, 0.0);

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
    public void testGetSetProgressDelta() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getProgressDelta(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_PROGRESS_DELTA, 0.0);

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
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getConfidence(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_CONFIDENCE, 0.0);

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
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getMaxIterations(),
                LMedSRobustKnownBiasTurntableGyroscopeCalibrator.DEFAULT_MAX_ITERATIONS);

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
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertTrue(calibrator.isResultRefined());

        // set new value
        calibrator.setResultRefined(false);

        // check
        assertFalse(calibrator.isResultRefined());
    }

    @Test
    public void testIsSetCovarianceKept() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertTrue(calibrator.isCovarianceKept());

        // set new value
        calibrator.setCovarianceKept(false);

        // check
        assertFalse(calibrator.isCovarianceKept());
    }

    @Test
    public void testGetSetQualityScores() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertNull(calibrator.getQualityScores());

        // set new value
        calibrator.setQualityScores(new double[3]);

        // check
        assertNull(calibrator.getQualityScores());
    }

    @Test
    public void testGetSetPreliminarySubsetSize() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getPreliminarySubsetSize(),
                TurntableGyroscopeCalibrator.MINIMUM_MEASUREMENTS_GENERAL_AND_CROSS_BIASES);

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
    public void testGetSetAccelerometerBiasX() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0, 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double bax = ba.getElementAtIndex(0);

        calibrator.setAccelerometerBiasX(bax);

        // check
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
    }

    @Test
    public void testGetSetAccelerometerBiasY() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0, 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double bay = ba.getElementAtIndex(1);

        calibrator.setAccelerometerBiasY(bay);

        // check
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
    }

    @Test
    public void testGetSetAccelerometerBiasZ() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0, 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double baz = ba.getElementAtIndex(2);

        calibrator.setAccelerometerBiasZ(baz);

        // check
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
    }

    @Test
    public void testGetSetAccelerometerBiasXAsAcceleration() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        final Acceleration acceleration1 = calibrator.getAccelerometerBiasXAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // set new value
        final Matrix ba = generateBa();
        final double bax = ba.getElementAtIndex(0);
        final Acceleration acceleration2 = new Acceleration(bax, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setAccelerometerBiasX(acceleration2);

        // check
        final Acceleration acceleration3 = calibrator.getAccelerometerBiasXAsAcceleration();
        final Acceleration acceleration4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasXAsAcceleration(acceleration4);
        assertEquals(acceleration2, acceleration3);
        assertEquals(acceleration2, acceleration4);
    }

    @Test
    public void testGetSetAccelerometerBiasYAsAcceleration() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        final Acceleration acceleration1 = calibrator.getAccelerometerBiasYAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // set new value
        final Matrix ba = generateBa();
        final double bay = ba.getElementAtIndex(1);
        final Acceleration acceleration2 = new Acceleration(bay, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setAccelerometerBiasY(acceleration2);

        // check
        final Acceleration acceleration3 = calibrator.getAccelerometerBiasYAsAcceleration();
        final Acceleration acceleration4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasYAsAcceleration(acceleration4);
        assertEquals(acceleration2, acceleration3);
        assertEquals(acceleration2, acceleration4);
    }

    @Test
    public void testGetSetAccelerometerBiasZAsAcceleration() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        final Acceleration acceleration1 = calibrator.getAccelerometerBiasZAsAcceleration();
        assertEquals(acceleration1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(acceleration1.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);

        // set new value
        final Matrix ba = generateBa();
        final double baz = ba.getElementAtIndex(2);
        final Acceleration acceleration2 = new Acceleration(baz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        calibrator.setAccelerometerBiasZ(acceleration2);

        // check
        final Acceleration acceleration3 = calibrator.getAccelerometerBiasZAsAcceleration();
        final Acceleration acceleration4 = new Acceleration(0.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        calibrator.getAccelerometerBiasZAsAcceleration(acceleration4);
        assertEquals(acceleration2, acceleration3);
        assertEquals(acceleration2, acceleration4);
    }

    @Test
    public void testSetAccelerometerBias1() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0, 0.0);


        // set new values
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
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getAccelerometerBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), 0.0, 0.0);

        // set new values
        final Matrix ba = generateBa();
        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        final Acceleration accelerationX = new Acceleration(bax,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(bay,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(baz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        calibrator.setAccelerometerBias(
                accelerationX, accelerationY, accelerationZ);

        // check
        assertEquals(calibrator.getAccelerometerBiasX(), bax, 0.0);
        assertEquals(calibrator.getAccelerometerBiasY(), bay, 0.0);
        assertEquals(calibrator.getAccelerometerBiasZ(), baz, 0.0);
    }

    @Test
    public void testGetSetAccelerometerBiasArray() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertArrayEquals(calibrator.getAccelerometerBias(), new double[3], 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double[] bias = ba.getBuffer();

        calibrator.setAccelerometerBias(bias);

        // check
        final double[] bias1 = calibrator.getAccelerometerBias();
        final double[] bias2 = new double[3];
        calibrator.getAccelerometerBias(bias2);

        assertArrayEquals(bias, bias1, 0.0);
        assertArrayEquals(bias, bias2, 0.0);

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
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerBiasAsMatrix(),
                new Matrix(3, 1));

        // set new value
        final Matrix ba = generateBa();
        calibrator.setAccelerometerBias(ba);

        // check
        final Matrix ba1 = calibrator.getAccelerometerBiasAsMatrix();
        final Matrix ba2 = new Matrix(3, 1);
        calibrator.getAccelerometerBiasAsMatrix(ba2);

        assertEquals(ba, ba1);
        assertEquals(ba, ba2);

        // Force IllegalArgumentException
        try {
            calibrator.getAccelerometerBiasAsMatrix(
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.getAccelerometerBiasAsMatrix(
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setAccelerometerBias(
                    new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            calibrator.setAccelerometerBias(
                    new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAccelerometerSx()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerSx(),
                0.0, 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double asx = ma.getElementAt(0, 0);

        calibrator.setAccelerometerSx(asx);

        // check
        assertEquals(calibrator.getAccelerometerSx(), asx, 0.0);
    }

    @Test
    public void testGetSetAccelerometerSy()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerSy(),
                0.0, 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double asy = ma.getElementAt(1, 1);

        calibrator.setAccelerometerSy(asy);

        // check
        assertEquals(calibrator.getAccelerometerSy(), asy, 0.0);
    }

    @Test
    public void testGetSetAccelerometerSz()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerSz(),
                0.0, 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double asz = ma.getElementAt(2, 2);

        calibrator.setAccelerometerSz(asz);

        // check
        assertEquals(calibrator.getAccelerometerSz(), asz, 0.0);
    }

    @Test
    public void testGetSetAccelerometerMxy()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerMxy(),
                0.0, 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double amxy = ma.getElementAt(0, 1);

        calibrator.setAccelerometerMxy(amxy);

        // check
        assertEquals(calibrator.getAccelerometerMxy(), amxy, 0.0);
    }

    @Test
    public void testGetSetAccelerometerMxz()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerMxz(),
                0.0, 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double amxz = ma.getElementAt(0, 2);

        calibrator.setAccelerometerMxz(amxz);

        // check
        assertEquals(calibrator.getAccelerometerMxz(), amxz, 0.0);
    }

    @Test
    public void testGetSetAccelerometerMyx()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerMyx(),
                0.0, 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double amyx = ma.getElementAt(1, 0);

        calibrator.setAccelerometerMyx(amyx);

        // check
        assertEquals(calibrator.getAccelerometerMyx(), amyx, 0.0);
    }

    @Test
    public void testGetSetAccelerometerMyz()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerMyz(),
                0.0, 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double amyz = ma.getElementAt(1, 2);

        calibrator.setAccelerometerMyz(amyz);

        // check
        assertEquals(calibrator.getAccelerometerMyz(), amyz, 0.0);
    }

    @Test
    public void testGetSetAccelerometerMzx()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerMzx(),
                0.0, 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double amzx = ma.getElementAt(2, 0);

        calibrator.setAccelerometerMzx(amzx);

        // check
        assertEquals(calibrator.getAccelerometerMzx(), amzx, 0.0);
    }

    @Test
    public void testGetSetAccelerometerMzy()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getAccelerometerMzy(),
                0.0, 0.0);

        // set new value
        final Matrix ma = generateMa();
        final double amzy = ma.getElementAt(2, 1);

        calibrator.setAccelerometerMzy(amzy);

        // check
        assertEquals(calibrator.getAccelerometerMzy(), amzy, 0.0);
    }

    @Test
    public void testGetSetAccelerometerScalingFactors()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getAccelerometerSx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), 0.0, 0.0);

        // set new values
        final Matrix ma = generateMa();
        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);

        calibrator.setAccelerometerScalingFactors(asx, asy, asz);

        // check
        assertEquals(calibrator.getAccelerometerSx(), asx, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), asy, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), asz, 0.0);
    }

    @Test
    public void testGetSetAccelerometerCrossCouplingErrors()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getAccelerometerMxy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), 0.0, 0.0);

        // set new values
        final Matrix ma = generateMa();
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        calibrator.setAccelerometerCrossCouplingErrors(
                amxy, amxz, amyx, amyz, amzx, amzy);

        // check
        assertEquals(calibrator.getAccelerometerMxy(), amxy, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), amxz, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), amyx, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), amyz, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), amzx, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), amzy, 0.0);
    }

    @Test
    public void testGetSetAccelerometerScalingFactorsAndCrossCouplingErrors()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getAccelerometerSx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), 0.0, 0.0);

        // set new values
        final Matrix ma = generateMa();
        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        calibrator.setAccelerometerScalingFactorsAndCrossCouplingErrors(
                asx, asy, asz, amxy, amxz, amyx, amyz, amzx, amzy);

        // check
        assertEquals(calibrator.getAccelerometerSx(), asx, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), asy, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), asz, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), amxy, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), amxz, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), amyx, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), amyz, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), amzx, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), amzy, 0.0);
    }

    @Test
    public void testGetSetAccelerometerMa() throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getAccelerometerSx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), 0.0, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), 0.0, 0.0);

        // set new values
        final Matrix ma = generateMa();
        final double asx = ma.getElementAt(0, 0);
        final double asy = ma.getElementAt(1, 1);
        final double asz = ma.getElementAt(2, 2);
        final double amxy = ma.getElementAt(0, 1);
        final double amxz = ma.getElementAt(0, 2);
        final double amyx = ma.getElementAt(1, 0);
        final double amyz = ma.getElementAt(1, 2);
        final double amzx = ma.getElementAt(2, 0);
        final double amzy = ma.getElementAt(2, 1);

        calibrator.setAccelerometerMa(ma);

        // check
        final Matrix ma1 = calibrator.getAccelerometerMa();
        final Matrix ma2 = new Matrix(3, 3);
        calibrator.getAccelerometerMa(ma2);

        assertEquals(ma, ma1);
        assertEquals(ma, ma2);

        assertEquals(calibrator.getAccelerometerSx(), asx, 0.0);
        assertEquals(calibrator.getAccelerometerSy(), asy, 0.0);
        assertEquals(calibrator.getAccelerometerSz(), asz, 0.0);
        assertEquals(calibrator.getAccelerometerMxy(), amxy, 0.0);
        assertEquals(calibrator.getAccelerometerMxz(), amxz, 0.0);
        assertEquals(calibrator.getAccelerometerMyx(), amyx, 0.0);
        assertEquals(calibrator.getAccelerometerMyz(), amyz, 0.0);
        assertEquals(calibrator.getAccelerometerMzx(), amzx, 0.0);
        assertEquals(calibrator.getAccelerometerMzy(), amzy, 0.0);

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
    public void testGetSetBiasX() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double bx = bg.getElementAtIndex(0);

        calibrator.setBiasX(bx);

        // check
        assertEquals(calibrator.getBiasX(), bx, 0.0);
    }

    @Test
    public void testGetSetBiasY() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double by = bg.getElementAtIndex(1);

        calibrator.setBiasY(by);

        // check
        assertEquals(calibrator.getBiasY(), by, 0.0);
    }

    @Test
    public void testGetSetBiasZ() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double bz = bg.getElementAtIndex(2);

        calibrator.setBiasZ(bz);

        // check
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
    }

    @Test
    public void testGetSetBiasAngularSpeedX()
            throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        final AngularSpeed angularSpeed1 = calibrator
                .getBiasAngularSpeedX();
        assertEquals(angularSpeed1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(angularSpeed1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);

        // set new value
        final Matrix bg = generateBg();
        final double bx = bg.getElementAtIndex(0);
        final AngularSpeed angularSpeed2 = new AngularSpeed(
                bx, AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setBiasX(angularSpeed2);

        // check
        final AngularSpeed angularSpeed3 = calibrator
                .getBiasAngularSpeedX();
        final AngularSpeed angularSpeed4 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedX(angularSpeed4);

        assertEquals(angularSpeed2, angularSpeed3);
        assertEquals(angularSpeed2, angularSpeed4);
    }

    @Test
    public void testGetSetBiasAngularSpeedY()
            throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        final AngularSpeed angularSpeed1 = calibrator
                .getBiasAngularSpeedY();
        assertEquals(angularSpeed1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(angularSpeed1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);

        // set new value
        final Matrix bg = generateBg();
        final double by = bg.getElementAtIndex(1);
        final AngularSpeed angularSpeed2 = new AngularSpeed(
                by, AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setBiasY(angularSpeed2);

        // check
        final AngularSpeed angularSpeed3 = calibrator
                .getBiasAngularSpeedY();
        final AngularSpeed angularSpeed4 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedY(angularSpeed4);

        assertEquals(angularSpeed2, angularSpeed3);
        assertEquals(angularSpeed2, angularSpeed4);
    }

    @Test
    public void testGetSetBiasAngularSpeedZ() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        final AngularSpeed angularSpeed1 = calibrator
                .getBiasAngularSpeedY();
        assertEquals(angularSpeed1.getValue().doubleValue(), 0.0, 0.0);
        assertEquals(angularSpeed1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);

        // set new value
        final Matrix bg = generateBg();
        final double bz = bg.getElementAtIndex(2);
        final AngularSpeed angularSpeed2 = new AngularSpeed(
                bz, AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setBiasZ(angularSpeed2);

        // check
        final AngularSpeed angularSpeed3 = calibrator
                .getBiasAngularSpeedZ();
        final AngularSpeed angularSpeed4 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getBiasAngularSpeedZ(angularSpeed4);

        assertEquals(angularSpeed2, angularSpeed3);
        assertEquals(angularSpeed2, angularSpeed4);
    }

    @Test
    public void testSetBias1() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);

        // set new values
        final Matrix bg = generateBg();
        final double bx = bg.getElementAtIndex(0);
        final double by = bg.getElementAtIndex(1);
        final double bz = bg.getElementAtIndex(2);

        calibrator.setBias(bx, by, bz);

        // check
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
    }

    @Test
    public void testSetBias2() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getBiasX(), 0.0, 0.0);
        assertEquals(calibrator.getBiasY(), 0.0, 0.0);
        assertEquals(calibrator.getBiasZ(), 0.0, 0.0);

        // set new values
        final Matrix bg = generateBg();
        final double bx = bg.getElementAtIndex(0);
        final double by = bg.getElementAtIndex(1);
        final double bz = bg.getElementAtIndex(2);

        final AngularSpeed angularSpeedX = new AngularSpeed(bx,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedY = new AngularSpeed(by,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed angularSpeedZ = new AngularSpeed(bz,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        calibrator.setBias(
                angularSpeedX, angularSpeedY, angularSpeedZ);

        // check
        assertEquals(calibrator.getBiasX(), bx, 0.0);
        assertEquals(calibrator.getBiasY(), by, 0.0);
        assertEquals(calibrator.getBiasZ(), bz, 0.0);
    }

    @Test
    public void testGetSetBiasAsTriad() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

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
    public void testGetSetInitialSx()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateCommonAxisMg();
        final double sx = mg.getElementAt(0, 0);

        calibrator.setInitialSx(sx);

        // check
        assertEquals(calibrator.getInitialSx(), sx, 0.0);
    }

    @Test
    public void testGetSetInitialSy()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateCommonAxisMg();
        final double sy = mg.getElementAt(1, 1);

        calibrator.setInitialSy(sy);

        // check
        assertEquals(calibrator.getInitialSy(), sy, 0.0);
    }

    @Test
    public void testGetSetInitialSz()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateCommonAxisMg();
        final double sz = mg.getElementAt(2, 2);

        calibrator.setInitialSz(sz);

        // check
        assertEquals(calibrator.getInitialSz(), sz, 0.0);
    }

    @Test
    public void testGetSetInitialMxy()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateCommonAxisMg();
        final double mxy = mg.getElementAt(0, 1);

        calibrator.setInitialMxy(mxy);

        // check
        assertEquals(calibrator.getInitialMxy(), mxy, 0.0);
    }

    @Test
    public void testGetSetInitialMxz()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateCommonAxisMg();
        final double mxz = mg.getElementAt(0, 2);

        calibrator.setInitialMxz(mxz);

        // check
        assertEquals(calibrator.getInitialMxz(), mxz, 0.0);
    }

    @Test
    public void testGetSetInitialMyx()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateCommonAxisMg();
        final double myx = mg.getElementAt(1, 0);

        calibrator.setInitialMyx(myx);

        // check
        assertEquals(calibrator.getInitialMyx(), myx, 0.0);
    }

    @Test
    public void testGetSetInitialMyz()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateCommonAxisMg();
        final double myz = mg.getElementAt(1, 2);

        calibrator.setInitialMyz(myz);

        // check
        assertEquals(calibrator.getInitialMyz(), myz, 0.0);
    }

    @Test
    public void testGetSetInitialMzx()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateCommonAxisMg();
        final double mzx = mg.getElementAt(2, 0);

        calibrator.setInitialMzx(mzx);

        // check
        assertEquals(calibrator.getInitialMzx(), mzx, 0.0);
    }

    @Test
    public void testGetSetInitialMzy()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        // set new value
        final Matrix mg = generateCommonAxisMg();
        final double mzy = mg.getElementAt(2, 1);

        calibrator.setInitialMzy(mzy);

        // check
        assertEquals(calibrator.getInitialMzy(), mzy, 0.0);
    }

    @Test
    public void testSetInitialScalingFactors()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getInitialSx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialSz(), 0.0, 0.0);

        // set new values
        final Matrix mg = generateCommonAxisMg();
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
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default values
        assertEquals(calibrator.getInitialMxy(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMxz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMyz(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzx(), 0.0, 0.0);
        assertEquals(calibrator.getInitialMzy(), 0.0, 0.0);

        // set new values
        final Matrix mg = generateCommonAxisMg();
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        calibrator.setInitialCrossCouplingErrors(
                mxy, mxz, myx, myz, mzx, mzy);

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
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

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
        final Matrix mg = generateCommonAxisMg();
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
    public void testGetSetBiasArray() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertArrayEquals(calibrator.getBias(), new double[3], 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double[] bias = bg.getBuffer();

        calibrator.setBias(bias);

        // check
        final double[] bias1 = calibrator.getBias();
        final double[] bias2 = new double[3];
        calibrator.getBias(bias2);

        assertArrayEquals(bias, bias1, 0.0);
        assertArrayEquals(bias, bias2, 0.0);

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
    public void testGetSetBiasAsMatrix()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getBiasAsMatrix(),
                new Matrix(3, 1));

        // set new value
        final Matrix bg = generateBg();
        final double bx = bg.getElementAtIndex(0);
        final double by = bg.getElementAtIndex(1);
        final double bz = bg.getElementAtIndex(2);

        calibrator.setBias(bg);

        // check
        final Matrix bg1 = calibrator.getBiasAsMatrix();
        final Matrix bg2 = new Matrix(3, 1);
        calibrator.getBiasAsMatrix(bg2);

        assertEquals(bg, bg1);
        assertEquals(bg, bg2);

        assertEquals(bx, calibrator.getBiasX(), 0.0);
        assertEquals(by, calibrator.getBiasY(), 0.0);
        assertEquals(bz, calibrator.getBiasZ(), 0.0);

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
    public void testGetSetInitialMg()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialMg(),
                new Matrix(3, 3));

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
        final Matrix mg = generateCommonAxisMg();
        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);

        calibrator.setInitialMg(mg);

        // check
        final Matrix mg1 = calibrator.getInitialMg();
        final Matrix mg2 = new Matrix(3, 3);
        calibrator.getInitialMg(mg2);

        assertEquals(mg, mg1);
        assertEquals(mg, mg2);

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
    public void testGetSetInitialGg()
            throws WrongSizeException, LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getInitialGg(),
                new Matrix(3, 3));

        // set new value
        final Matrix gg = generateGg();
        calibrator.setInitialGg(gg);

        // check
        final Matrix gg1 = calibrator.getInitialGg();
        final Matrix gg2 = new Matrix(3, 3);
        calibrator.getInitialGg(gg2);

        assertEquals(gg, gg1);
        assertEquals(gg, gg2);

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
    public void testGetSetTurntableRotationRate() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getTurntableRotationRate(),
                Constants.EARTH_ROTATION_RATE, 0.0);

        // set new value
        final double rotationRate = getTurntableRotationRate();
        calibrator.setTurntableRotationRate(rotationRate);

        // check
        assertEquals(calibrator.getTurntableRotationRate(), rotationRate,
                0.0);

        // Force IllegalArgumentException
        try {
            calibrator.setTurntableRotationRate(0.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetTurntableRotationRateAsAngularSpeed()
            throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        final AngularSpeed rotationRate1 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        assertEquals(rotationRate1.getValue().doubleValue(),
                Constants.EARTH_ROTATION_RATE, 0.0);
        assertEquals(rotationRate1.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);

        // set new value
        final double rotationRate = getTurntableRotationRate();
        final AngularSpeed rotationRate2 = new AngularSpeed(rotationRate,
                AngularSpeedUnit.RADIANS_PER_SECOND);
        calibrator.setTurntableRotationRate(rotationRate2);

        // check
        final AngularSpeed rotation3 = calibrator
                .getTurntableRotationRateAsAngularSpeed();
        final AngularSpeed rotation4 = new AngularSpeed(0.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        calibrator.getTurntableRotationRateAsAngularSpeed(rotation4);

        assertEquals(rotation3, rotation4);

        // Force IllegalArgumentException
        try {
            calibrator.setTurntableRotationRate(
                    new AngularSpeed(0.0,
                            AngularSpeedUnit.RADIANS_PER_SECOND));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetMeasurements() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

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
    public void testGetSetEcefPosition() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertNull(calibrator.getEcefPosition());

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
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
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertNull(calibrator.getNedPosition());
        assertFalse(calibrator.getNedPosition(null));

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

        calibrator.setPosition(nedPosition);

        // check
        assertTrue(calibrator.getNedPosition().equals(nedPosition,
                LARGE_ABSOLUTE_ERROR));
        final NEDPosition position2 = new NEDPosition();
        assertTrue(calibrator.getNedPosition(position2));
        assertTrue(nedPosition.equals(position2, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testIsSetCommonAxisUsed() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertTrue(calibrator.isCommonAxisUsed());

        // set new value
        calibrator.setCommonAxisUsed(false);

        // check
        assertFalse(calibrator.isCommonAxisUsed());
    }

    @Test
    public void testIsSetGDependentCrossBiasesEstimated()
            throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());

        // set new value
        calibrator.setGDependentCrossBiasesEstimated(false);

        // check
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertNull(calibrator.getListener());

        // set new value
        calibrator.setListener(this);

        // check
        assertSame(calibrator.getListener(), this);
    }

    @Test
    public void testGetMinimumRequiredMeasurements() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                16);
        assertTrue(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());

        // set new value
        calibrator.setGDependentCrossBiasesEstimated(false);

        // check
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                7);
        assertTrue(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());

        // set new value
        calibrator.setCommonAxisUsed(false);

        // check
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                10);
        assertFalse(calibrator.isCommonAxisUsed());
        assertFalse(calibrator.isGDependentCrossBiasesEstimated());

        // set new value
        calibrator.setGDependentCrossBiasesEstimated(true);

        // check
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                19);
        assertFalse(calibrator.isCommonAxisUsed());
        assertTrue(calibrator.isGDependentCrossBiasesEstimated());
    }

    @Test
    public void testIsReady() throws LockedException {
        final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                new LMedSRobustKnownBiasTurntableGyroscopeCalibrator();

        // check default value
        assertFalse(calibrator.isReady());
        assertEquals(calibrator.getMinimumRequiredMeasurements(),
                16);

        // set empty measurements
        final List<StandardDeviationBodyKinematics> measurements =
                new ArrayList<>();
        calibrator.setMeasurements(measurements);

        // check
        assertFalse(calibrator.isReady());

        // set enough measurements
        for (int i = 0;
             i < KnownBiasTurntableGyroscopeCalibrator.MINIMUM_MEASUREMENTS_COMMON_Z_AXIS_AND_CROSS_BIASES; i++) {
            measurements.add(new StandardDeviationBodyKinematics());
        }

        // check
        assertTrue(calibrator.isReady());
    }

    @Test
    public void testCalibrateCommonAxisAndGDependentCrossBiasesDisabledAndNoInlierNoise()
            throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, InvalidRotationMatrixException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final double rotationRate = 100.0 * getTurntableRotationRate();
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
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            double timeInterval = TIME_INTERVAL_SECONDS;
            final List<StandardDeviationBodyKinematics> measurements =
                    new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                final double roll1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final CoordinateTransformation nedC1 = new CoordinateTransformation(
                        roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame1);

                final Rotation3D rot1 = nedC1.asRotation();
                final double[] axis1 = rot1.getRotationAxis();
                double angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final double x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final Rotation3D rot = new AxisRotation3D(axis1, angleIncrement);
                final Rotation3D rot2 = rot1.combineAndReturnNew(rot);
                final CoordinateTransformation nedC2 =
                        new CoordinateTransformation(rot2.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(timeInterval, ecefFrame2,
                                ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final BodyKinematics measuredKinematics;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(timeInterval, trueKinematics, errorsOutlier, random);

                } else {
                    // inlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(timeInterval, trueKinematics, errorsInlier, random);
                }

                final StandardDeviationBodyKinematics measurement =
                        new StandardDeviationBodyKinematics(measuredKinematics,
                                specificForceStandardDeviation,
                                angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                    new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                            rotationRate, timeInterval,
                            measurements,
                            true,
                            false,
                            bg, mg, gg, this);
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

            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralAndGDependentCrossBiasesDisabledAndNoInlierNoise()
            throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, InvalidRotationMatrixException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final double rotationRate = 100.0 * getTurntableRotationRate();
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
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            double timeInterval = TIME_INTERVAL_SECONDS;
            final List<StandardDeviationBodyKinematics> measurements =
                    new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                final double roll1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final CoordinateTransformation nedC1 = new CoordinateTransformation(
                        roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame1);

                final Rotation3D rot1 = nedC1.asRotation();
                final double[] axis1 = rot1.getRotationAxis();
                double angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final double x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final Rotation3D rot = new AxisRotation3D(axis1, angleIncrement);
                final Rotation3D rot2 = rot1.combineAndReturnNew(rot);
                final CoordinateTransformation nedC2 =
                        new CoordinateTransformation(rot2.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(timeInterval, ecefFrame2,
                                ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final BodyKinematics measuredKinematics;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(timeInterval, trueKinematics, errorsOutlier, random);

                } else {
                    // inlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(timeInterval, trueKinematics, errorsInlier, random);
                }

                final StandardDeviationBodyKinematics measurement =
                        new StandardDeviationBodyKinematics(measuredKinematics,
                                specificForceStandardDeviation,
                                angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                    new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                            rotationRate, timeInterval,
                            measurements,
                            false,
                            false,
                            bg, mg, gg, this);
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

            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateCommonAxisAndGDependentCrossBiasesEnabledAndNoInlierNoise()
            throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, InvalidRotationMatrixException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final double rotationRate = 100.0 * getTurntableRotationRate();
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
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            double timeInterval = TIME_INTERVAL_SECONDS;
            final List<StandardDeviationBodyKinematics> measurements =
                    new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                final double roll1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final CoordinateTransformation nedC1 = new CoordinateTransformation(
                        roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame1);

                final Rotation3D rot1 = nedC1.asRotation();
                final double[] axis1 = rot1.getRotationAxis();
                double angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final double x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final Rotation3D rot = new AxisRotation3D(axis1, angleIncrement);
                final Rotation3D rot2 = rot1.combineAndReturnNew(rot);
                final CoordinateTransformation nedC2 =
                        new CoordinateTransformation(rot2.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(timeInterval, ecefFrame2,
                                ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final BodyKinematics measuredKinematics;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(timeInterval, trueKinematics, errorsOutlier, random);

                } else {
                    // inlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(timeInterval, trueKinematics, errorsInlier, random);
                }

                final StandardDeviationBodyKinematics measurement =
                        new StandardDeviationBodyKinematics(measuredKinematics,
                                specificForceStandardDeviation,
                                angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                    new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                            rotationRate, timeInterval,
                            measurements,
                            true,
                            true,
                            bg, mg, gg, ba, ma, this);
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

            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralAndGDependentCrossBiasesEnabledAndNoInlierNoise()
            throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, InvalidRotationMatrixException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final double rotationRate = 100.0 * getTurntableRotationRate();
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
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            double timeInterval = TIME_INTERVAL_SECONDS;
            final List<StandardDeviationBodyKinematics> measurements =
                    new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                final double roll1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final CoordinateTransformation nedC1 = new CoordinateTransformation(
                        roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame1);

                final Rotation3D rot1 = nedC1.asRotation();
                final double[] axis1 = rot1.getRotationAxis();
                double angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final double x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final Rotation3D rot = new AxisRotation3D(axis1, angleIncrement);
                final Rotation3D rot2 = rot1.combineAndReturnNew(rot);
                final CoordinateTransformation nedC2 =
                        new CoordinateTransformation(rot2.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(timeInterval, ecefFrame2,
                                ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final BodyKinematics measuredKinematics;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(timeInterval, trueKinematics, errorsOutlier, random);

                } else {
                    // inlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(timeInterval, trueKinematics, errorsInlier, random);
                }

                final StandardDeviationBodyKinematics measurement =
                        new StandardDeviationBodyKinematics(measuredKinematics,
                                specificForceStandardDeviation,
                                angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                    new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                            rotationRate, timeInterval,
                            measurements,
                            false,
                            true,
                            bg, mg, gg, ba, ma, this);
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
    public void testCalibrateCommonAxisAndGDependentCrossBiasesDisabledWithInlierNoise()
            throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, InvalidRotationMatrixException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final double rotationRate = 100.0 * getTurntableRotationRate();
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

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            double timeInterval = TIME_INTERVAL_SECONDS;
            final List<StandardDeviationBodyKinematics> measurements =
                    new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                final double roll1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final CoordinateTransformation nedC1 = new CoordinateTransformation(
                        roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame1);

                final Rotation3D rot1 = nedC1.asRotation();
                final double[] axis1 = rot1.getRotationAxis();
                double angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final double x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final Rotation3D rot = new AxisRotation3D(axis1, angleIncrement);
                final Rotation3D rot2 = rot1.combineAndReturnNew(rot);
                final CoordinateTransformation nedC2 =
                        new CoordinateTransformation(rot2.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(timeInterval, ecefFrame2,
                                ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final BodyKinematics measuredKinematics;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(timeInterval, trueKinematics, errorsOutlier, random);

                } else {
                    // inlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(timeInterval, trueKinematics, errorsInlier, random);
                }

                final StandardDeviationBodyKinematics measurement =
                        new StandardDeviationBodyKinematics(measuredKinematics,
                                specificForceStandardDeviation,
                                angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                    new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                            rotationRate, timeInterval,
                            measurements,
                            true,
                            false,
                            bg, mg, gg, this);
            calibrator.setStopThreshold(LARGE_THRESHOLD);

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

            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralAndGDependentCrossBiasesDisabledWithInlierNoise()
            throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, InvalidRotationMatrixException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final double rotationRate = 100.0 * getTurntableRotationRate();
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

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            double timeInterval = TIME_INTERVAL_SECONDS;
            final List<StandardDeviationBodyKinematics> measurements =
                    new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                final double roll1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final CoordinateTransformation nedC1 = new CoordinateTransformation(
                        roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame1);

                final Rotation3D rot1 = nedC1.asRotation();
                final double[] axis1 = rot1.getRotationAxis();
                double angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final double x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final Rotation3D rot = new AxisRotation3D(axis1, angleIncrement);
                final Rotation3D rot2 = rot1.combineAndReturnNew(rot);
                final CoordinateTransformation nedC2 =
                        new CoordinateTransformation(rot2.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(timeInterval, ecefFrame2,
                                ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final BodyKinematics measuredKinematics;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(timeInterval, trueKinematics, errorsOutlier, random);

                } else {
                    // inlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(timeInterval, trueKinematics, errorsInlier, random);
                }

                final StandardDeviationBodyKinematics measurement =
                        new StandardDeviationBodyKinematics(measuredKinematics,
                                specificForceStandardDeviation,
                                angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                    new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                            rotationRate, timeInterval,
                            measurements,
                            false,
                            false,
                            bg, mg, gg, this);
            calibrator.setStopThreshold(LARGE_THRESHOLD);

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

            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            assertNotNull(calibrator.getEstimatedCovariance());

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateCommonAxisAndGDependentCrossBiasesEnabledWithInlierNoise()
            throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, InvalidRotationMatrixException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final double rotationRate = 100.0 * getTurntableRotationRate();
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

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            double timeInterval = TIME_INTERVAL_SECONDS;
            final List<StandardDeviationBodyKinematics> measurements =
                    new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                final double roll1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final CoordinateTransformation nedC1 = new CoordinateTransformation(
                        roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame1);

                final Rotation3D rot1 = nedC1.asRotation();
                final double[] axis1 = rot1.getRotationAxis();
                double angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final double x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final Rotation3D rot = new AxisRotation3D(axis1, angleIncrement);
                final Rotation3D rot2 = rot1.combineAndReturnNew(rot);
                final CoordinateTransformation nedC2 =
                        new CoordinateTransformation(rot2.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(timeInterval, ecefFrame2,
                                ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final BodyKinematics measuredKinematics;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(timeInterval, trueKinematics, errorsOutlier, random);

                } else {
                    // inlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(timeInterval, trueKinematics, errorsInlier, random);
                }

                final StandardDeviationBodyKinematics measurement =
                        new StandardDeviationBodyKinematics(measuredKinematics,
                                specificForceStandardDeviation,
                                angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                    new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                            rotationRate, timeInterval,
                            measurements,
                            true,
                            true,
                            bg, mg, gg, ba, ma, this);
            calibrator.setStopThreshold(LARGE_THRESHOLD);

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

            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mg.equals(estimatedMg, LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testCalibrateGeneralAndGDependentCrossBiasesEnabledWithInlierNoise()
            throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, InvalidRotationMatrixException, RotationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final double rotationRate = 100.0 * getTurntableRotationRate();
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

            final Random random = new Random();
            final UniformRandomizer randomizer = new UniformRandomizer(random);
            final double latitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREEs));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
            final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
            final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

            double timeInterval = TIME_INTERVAL_SECONDS;
            final List<StandardDeviationBodyKinematics> measurements =
                    new ArrayList<>();
            for (int i = 0; i < MEASUREMENT_NUMBER; i++) {
                final double roll1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double pitch1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
                final double yaw1 = Math.toRadians(
                        randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

                final CoordinateTransformation nedC1 = new CoordinateTransformation(
                        roll1, pitch1, yaw1, FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC1);
                final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame1);

                final Rotation3D rot1 = nedC1.asRotation();
                final double[] axis1 = rot1.getRotationAxis();
                double angleIncrement = rotationRate * timeInterval;
                if (Math.abs(angleIncrement) > Math.PI / 2.0) {
                    // angle = rot_rate * interval
                    // rot_rate * interval / x = angle / x

                    // if we want angle / x = pi / 2, then:
                    final double x = Math.abs(angleIncrement) / (Math.PI / 2.0);
                    timeInterval /= x;
                    angleIncrement = rotationRate * timeInterval;
                }
                final Rotation3D rot = new AxisRotation3D(axis1, angleIncrement);
                final Rotation3D rot2 = rot1.combineAndReturnNew(rot);
                final CoordinateTransformation nedC2 =
                        new CoordinateTransformation(rot2.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                                FrameType.LOCAL_NAVIGATION_FRAME);

                final NEDFrame nedFrame2 = new NEDFrame(nedPosition, nedC2);
                final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                        .convertNEDtoECEFAndReturnNew(nedFrame2);

                // compute ground-truth kinematics that should be generated at provided
                // position, velocity and orientation
                final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                        .estimateKinematicsAndReturnNew(timeInterval, ecefFrame2,
                                ecefFrame1);

                // apply known calibration parameters to distort ground-truth and generate a
                // measured kinematics sample
                final BodyKinematics measuredKinematics;
                if (randomizer.nextInt(0, 100) < OUTLIER_PERCENTAGE) {
                    // outlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(timeInterval, trueKinematics, errorsOutlier, random);

                } else {
                    // inlier
                    measuredKinematics = BodyKinematicsGenerator
                            .generate(timeInterval, trueKinematics, errorsInlier, random);
                }

                final StandardDeviationBodyKinematics measurement =
                        new StandardDeviationBodyKinematics(measuredKinematics,
                                specificForceStandardDeviation,
                                angularRateStandardDeviation);
                measurements.add(measurement);
            }

            // When we have the minimum number of measurements, we need to provide
            // an initial solution close to the true solution
            final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator =
                    new LMedSRobustKnownBiasTurntableGyroscopeCalibrator(nedPosition,
                            rotationRate, timeInterval,
                            measurements,
                            false,
                            true,
                            bg, mg, gg, ba, ma, this);
            calibrator.setStopThreshold(LARGE_THRESHOLD);

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

            final Matrix estimatedMg = calibrator.getEstimatedMg();
            final Matrix estimatedGg = calibrator.getEstimatedGg();

            if (!mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, VERY_LARGE_ABSOLUTE_ERROR));

            assertEstimatedResult(estimatedMg, estimatedGg, calibrator);

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onCalibrateStart(final RobustKnownBiasTurntableGyroscopeCalibrator calibrator) {
        checkLocked((LMedSRobustKnownBiasTurntableGyroscopeCalibrator) calibrator);
        mCalibrateStart++;
    }

    @Override
    public void onCalibrateEnd(
            final RobustKnownBiasTurntableGyroscopeCalibrator calibrator) {
        checkLocked((LMedSRobustKnownBiasTurntableGyroscopeCalibrator) calibrator);
        mCalibrateEnd++;
    }

    @Override
    public void onCalibrateNextIteration(
            final RobustKnownBiasTurntableGyroscopeCalibrator calibrator,
            final int iteration) {
        checkLocked((LMedSRobustKnownBiasTurntableGyroscopeCalibrator) calibrator);
        mCalibrateNextIteration++;
    }

    @Override
    public void onCalibrateProgressChange(
            final RobustKnownBiasTurntableGyroscopeCalibrator calibrator,
            final float progress) {
        checkLocked((LMedSRobustKnownBiasTurntableGyroscopeCalibrator) calibrator);
        mCalibrateProgressChange++;
    }

    private void reset() {
        mCalibrateStart = 0;
        mCalibrateEnd = 0;
        mCalibrateNextIteration = 0;
        mCalibrateProgressChange = 0;
    }

    private void checkLocked(final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator) {
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
                    0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerBias(
                    null, null, null);
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
                    0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerCrossCouplingErrors(
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerScalingFactorsAndCrossCouplingErrors(
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setAccelerometerMa(null);
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
            calibrator.setBias(0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setBias(null, null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setBias((AngularSpeedTriad) null);
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
            calibrator.setTurntableRotationRate(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setTurntableRotationRate(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setTimeInterval(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setTimeInterval(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setMeasurements(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setPosition((ECEFPosition) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setPosition((NEDPosition) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setCommonAxisUsed(true);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            calibrator.setGDependentCrossBiasesEstimated(true);
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
            calibrator.setPreliminarySubsetSize(30);
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
    }

    private void assertEstimatedResult(
            final Matrix mg, final Matrix gg,
            final LMedSRobustKnownBiasTurntableGyroscopeCalibrator calibrator) {

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
                -1000 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                800 * MICRO_G_TO_METERS_PER_SECOND_SQUARED});
    }

    private Matrix generateBg() {
        return Matrix.newFromArray(new double[]{
                -9 * DEG_TO_RAD / 3600.0,
                10 * DEG_TO_RAD / 3600.0,
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

    private double getTurntableRotationRate() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        return Math.toRadians(randomizer.nextDouble(
                MIN_ROTATION_RATE_DEGREES_PER_SECOND, MAX_ROTATION_RATE_DEGREES_PER_SECOND));
    }

}
