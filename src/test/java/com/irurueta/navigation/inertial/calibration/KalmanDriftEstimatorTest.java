/*
 * Copyright (C) 2021 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.geometry.RotationException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.*;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownGravityNormAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerAndGyroscopeMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.gyroscope.EasyGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionIntegrator;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource;
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.BracketedAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer;
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.IntervalDetectorThresholdFactorOptimizerException;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator;
import com.irurueta.navigation.inertial.navigators.InertialNavigatorException;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.*;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.Assert.*;

public class KalmanDriftEstimatorTest implements DriftEstimatorListener {

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

    // number of samples is equivalent to 30s for default time interval
    private static final int N_SAMPLES = 1500;

    private static final double FRAME_ABSOLUTE_ERROR = 1e-8;

    private static final double ABSOLUTE_ERROR = 1e-12;

    private static final double MIN_DELTA_POS_METERS = -1e-3;
    private static final double MAX_DELTA_POS_METERS = 1e-3;
    private static final double MIN_DELTA_ANGLE_DEGREES = -2.0;
    private static final double MAX_DELTA_ANGLE_DEGREES = 2.0;

    private static final int TIMES = 100;

    private static final Logger LOGGER = Logger.getLogger(
            DriftEstimatorTest.class.getName());

    private int mStart;
    private int mBodyKinematicsAdded;
    private int mReset;

    private final List<TimedBodyKinematics> mTimedBodyKinematics = new ArrayList<>();

    private final AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource mDataSource =
            new AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource() {

                @Override
                public int count() {
                    return mTimedBodyKinematics.size();
                }

                @Override
                public TimedBodyKinematics getAt(final int index) {
                    return mTimedBodyKinematics.get(index);
                }
            };

    @Test
    public void testConstructor1() throws WrongSizeException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default values
        assertNull(estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(new Matrix(3, 1),
                estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(new Matrix(3, 1), ba1);
        assertArrayEquals(new double[3], estimator.getAccelerationBiasArray(),
                0.0);
        final double[] ba2 = new double[3];
        estimator.getAccelerationBiasArray(ba2);
        assertArrayEquals(new double[3], ba2, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, baTriad1.getValueX(), 0.0);
        assertEquals(0.0, baTriad1.getValueY(), 0.0);
        assertEquals(0.0, baTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baTriad1.getUnit());
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad1, baTriad2);
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        assertEquals(new Matrix(3, 3),
                estimator.getAccelerationCrossCouplingErrors());
        final Matrix ma = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma);
        assertEquals(new Matrix(3, 3), ma);
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);
        assertEquals(new Matrix(3, 1),
                estimator.getAngularSpeedBias());
        final Matrix bg1 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg1);
        assertEquals(new Matrix(3, 1), bg1);
        assertArrayEquals(new double[3], estimator.getAngularSpeedBiasArray(),
                0.0);
        final double[] bg2 = new double[3];
        estimator.getAngularSpeedBiasArray(bg2);
        assertArrayEquals(new double[3], bg2, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, bgTriad1.getValueX(), 0.0);
        assertEquals(0.0, bgTriad1.getValueY(), 0.0);
        assertEquals(0.0, bgTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgTriad1.getUnit());
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad1, bgTriad2);
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedCrossCouplingErrors());
        final Matrix mg = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg);
        assertEquals(new Matrix(3, 3), mg);
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());
        final Matrix gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertNull(estimator.getKalmanConfig());
        assertNull(estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
    }

    @Test
    public void testConstructor2() throws WrongSizeException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator(this);

        // check default values
        assertSame(this, estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(new Matrix(3, 1),
                estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(new Matrix(3, 1), ba1);
        assertArrayEquals(new double[3], estimator.getAccelerationBiasArray(),
                0.0);
        final double[] ba2 = new double[3];
        estimator.getAccelerationBiasArray(ba2);
        assertArrayEquals(new double[3], ba2, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, baTriad1.getValueX(), 0.0);
        assertEquals(0.0, baTriad1.getValueY(), 0.0);
        assertEquals(0.0, baTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baTriad1.getUnit());
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad1, baTriad2);
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        assertEquals(new Matrix(3, 3),
                estimator.getAccelerationCrossCouplingErrors());
        final Matrix ma = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma);
        assertEquals(new Matrix(3, 3), ma);
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);
        assertEquals(new Matrix(3, 1),
                estimator.getAngularSpeedBias());
        final Matrix bg1 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg1);
        assertEquals(new Matrix(3, 1), bg1);
        assertArrayEquals(new double[3], estimator.getAngularSpeedBiasArray(),
                0.0);
        final double[] bg2 = new double[3];
        estimator.getAngularSpeedBiasArray(bg2);
        assertArrayEquals(new double[3], bg2, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, bgTriad1.getValueX(), 0.0);
        assertEquals(0.0, bgTriad1.getValueY(), 0.0);
        assertEquals(0.0, bgTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgTriad1.getUnit());
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad1, bgTriad2);
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedCrossCouplingErrors());
        final Matrix mg = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg);
        assertEquals(new Matrix(3, 3), mg);
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());
        final Matrix gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertNull(estimator.getKalmanConfig());
        assertNull(estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
    }

    @Test
    public void testConstructor3() throws WrongSizeException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator(
                kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(new Matrix(3, 1),
                estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(new Matrix(3, 1), ba1);
        assertArrayEquals(new double[3], estimator.getAccelerationBiasArray(),
                0.0);
        final double[] ba2 = new double[3];
        estimator.getAccelerationBiasArray(ba2);
        assertArrayEquals(new double[3], ba2, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, baTriad1.getValueX(), 0.0);
        assertEquals(0.0, baTriad1.getValueY(), 0.0);
        assertEquals(0.0, baTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baTriad1.getUnit());
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad1, baTriad2);
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        assertEquals(new Matrix(3, 3),
                estimator.getAccelerationCrossCouplingErrors());
        final Matrix ma = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma);
        assertEquals(new Matrix(3, 3), ma);
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);
        assertEquals(new Matrix(3, 1),
                estimator.getAngularSpeedBias());
        final Matrix bg1 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg1);
        assertEquals(new Matrix(3, 1), bg1);
        assertArrayEquals(new double[3], estimator.getAngularSpeedBiasArray(),
                0.0);
        final double[] bg2 = new double[3];
        estimator.getAngularSpeedBiasArray(bg2);
        assertArrayEquals(new double[3], bg2, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, bgTriad1.getValueX(), 0.0);
        assertEquals(0.0, bgTriad1.getValueY(), 0.0);
        assertEquals(0.0, bgTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgTriad1.getUnit());
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad1, bgTriad2);
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedCrossCouplingErrors());
        final Matrix mg = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg);
        assertEquals(new Matrix(3, 3), mg);
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());
        final Matrix gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
    }

    @Test
    public void testConstructor4() throws WrongSizeException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final KalmanDriftEstimator estimator = new KalmanDriftEstimator(
                kalmanConfig, initConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(new Matrix(3, 1),
                estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(new Matrix(3, 1), ba1);
        assertArrayEquals(new double[3], estimator.getAccelerationBiasArray(),
                0.0);
        final double[] ba2 = new double[3];
        estimator.getAccelerationBiasArray(ba2);
        assertArrayEquals(new double[3], ba2, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, baTriad1.getValueX(), 0.0);
        assertEquals(0.0, baTriad1.getValueY(), 0.0);
        assertEquals(0.0, baTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baTriad1.getUnit());
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad1, baTriad2);
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        assertEquals(new Matrix(3, 3),
                estimator.getAccelerationCrossCouplingErrors());
        final Matrix ma = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma);
        assertEquals(new Matrix(3, 3), ma);
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);
        assertEquals(new Matrix(3, 1),
                estimator.getAngularSpeedBias());
        final Matrix bg1 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg1);
        assertEquals(new Matrix(3, 1), bg1);
        assertArrayEquals(new double[3], estimator.getAngularSpeedBiasArray(),
                0.0);
        final double[] bg2 = new double[3];
        estimator.getAngularSpeedBiasArray(bg2);
        assertArrayEquals(new double[3], bg2, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, bgTriad1.getValueX(), 0.0);
        assertEquals(0.0, bgTriad1.getValueY(), 0.0);
        assertEquals(0.0, bgTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgTriad1.getUnit());
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad1, bgTriad2);
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedCrossCouplingErrors());
        final Matrix mg = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg);
        assertEquals(new Matrix(3, 3), mg);
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());
        final Matrix gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
    }

    @Test
    public void testConstructor5() throws WrongSizeException {
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final KalmanDriftEstimator estimator = new KalmanDriftEstimator(ecefFrame,
                kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertSame(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final CoordinateTransformation ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final CoordinateTransformation nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(new Matrix(3, 1),
                estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(new Matrix(3, 1), ba1);
        assertArrayEquals(new double[3], estimator.getAccelerationBiasArray(),
                0.0);
        final double[] ba2 = new double[3];
        estimator.getAccelerationBiasArray(ba2);
        assertArrayEquals(new double[3], ba2, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, baTriad1.getValueX(), 0.0);
        assertEquals(0.0, baTriad1.getValueY(), 0.0);
        assertEquals(0.0, baTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baTriad1.getUnit());
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad1, baTriad2);
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        assertEquals(new Matrix(3, 3),
                estimator.getAccelerationCrossCouplingErrors());
        final Matrix ma = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma);
        assertEquals(new Matrix(3, 3), ma);
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);
        assertEquals(new Matrix(3, 1),
                estimator.getAngularSpeedBias());
        final Matrix bg1 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg1);
        assertEquals(new Matrix(3, 1), bg1);
        assertArrayEquals(new double[3], estimator.getAngularSpeedBiasArray(),
                0.0);
        final double[] bg2 = new double[3];
        estimator.getAngularSpeedBiasArray(bg2);
        assertArrayEquals(new double[3], bg2, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, bgTriad1.getValueX(), 0.0);
        assertEquals(0.0, bgTriad1.getValueY(), 0.0);
        assertEquals(0.0, bgTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgTriad1.getUnit());
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad1, bgTriad2);
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedCrossCouplingErrors());
        final Matrix mg = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg);
        assertEquals(new Matrix(3, 3), mg);
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());
        final Matrix gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
    }

    @Test
    public void testConstructor6() throws WrongSizeException {
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final KalmanDriftEstimator estimator = new KalmanDriftEstimator(ecefFrame,
                kalmanConfig, initConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertSame(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final CoordinateTransformation ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final CoordinateTransformation nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(new Matrix(3, 1),
                estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(new Matrix(3, 1), ba1);
        assertArrayEquals(new double[3], estimator.getAccelerationBiasArray(),
                0.0);
        final double[] ba2 = new double[3];
        estimator.getAccelerationBiasArray(ba2);
        assertArrayEquals(new double[3], ba2, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, baTriad1.getValueX(), 0.0);
        assertEquals(0.0, baTriad1.getValueY(), 0.0);
        assertEquals(0.0, baTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baTriad1.getUnit());
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad1, baTriad2);
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        assertEquals(new Matrix(3, 3),
                estimator.getAccelerationCrossCouplingErrors());
        final Matrix ma = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma);
        assertEquals(new Matrix(3, 3), ma);
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);
        assertEquals(new Matrix(3, 1),
                estimator.getAngularSpeedBias());
        final Matrix bg1 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg1);
        assertEquals(new Matrix(3, 1), bg1);
        assertArrayEquals(new double[3], estimator.getAngularSpeedBiasArray(),
                0.0);
        final double[] bg2 = new double[3];
        estimator.getAngularSpeedBiasArray(bg2);
        assertArrayEquals(new double[3], bg2, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, bgTriad1.getValueX(), 0.0);
        assertEquals(0.0, bgTriad1.getValueY(), 0.0);
        assertEquals(0.0, bgTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgTriad1.getUnit());
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad1, bgTriad2);
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedCrossCouplingErrors());
        final Matrix mg = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg);
        assertEquals(new Matrix(3, 3), mg);
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());
        final Matrix gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
    }

    @Test
    public void testConstructor7() throws WrongSizeException {
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final KalmanDriftEstimator estimator = new KalmanDriftEstimator(nedFrame,
                kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final CoordinateTransformation ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final CoordinateTransformation nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(new Matrix(3, 1),
                estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(new Matrix(3, 1), ba1);
        assertArrayEquals(new double[3], estimator.getAccelerationBiasArray(),
                0.0);
        final double[] ba2 = new double[3];
        estimator.getAccelerationBiasArray(ba2);
        assertArrayEquals(new double[3], ba2, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, baTriad1.getValueX(), 0.0);
        assertEquals(0.0, baTriad1.getValueY(), 0.0);
        assertEquals(0.0, baTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baTriad1.getUnit());
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad1, baTriad2);
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        assertEquals(new Matrix(3, 3),
                estimator.getAccelerationCrossCouplingErrors());
        final Matrix ma = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma);
        assertEquals(new Matrix(3, 3), ma);
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);
        assertEquals(new Matrix(3, 1),
                estimator.getAngularSpeedBias());
        final Matrix bg1 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg1);
        assertEquals(new Matrix(3, 1), bg1);
        assertArrayEquals(new double[3], estimator.getAngularSpeedBiasArray(),
                0.0);
        final double[] bg2 = new double[3];
        estimator.getAngularSpeedBiasArray(bg2);
        assertArrayEquals(new double[3], bg2, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, bgTriad1.getValueX(), 0.0);
        assertEquals(0.0, bgTriad1.getValueY(), 0.0);
        assertEquals(0.0, bgTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgTriad1.getUnit());
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad1, bgTriad2);
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedCrossCouplingErrors());
        final Matrix mg = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg);
        assertEquals(new Matrix(3, 3), mg);
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());
        final Matrix gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
    }

    @Test
    public void testConstructor8() throws WrongSizeException {
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final KalmanDriftEstimator estimator = new KalmanDriftEstimator(nedFrame,
                kalmanConfig, initConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final CoordinateTransformation ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final CoordinateTransformation nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(new Matrix(3, 1),
                estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(new Matrix(3, 1), ba1);
        assertArrayEquals(new double[3], estimator.getAccelerationBiasArray(),
                0.0);
        final double[] ba2 = new double[3];
        estimator.getAccelerationBiasArray(ba2);
        assertArrayEquals(new double[3], ba2, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, baTriad1.getValueX(), 0.0);
        assertEquals(0.0, baTriad1.getValueY(), 0.0);
        assertEquals(0.0, baTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baTriad1.getUnit());
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad1, baTriad2);
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        assertEquals(new Matrix(3, 3),
                estimator.getAccelerationCrossCouplingErrors());
        final Matrix ma = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma);
        assertEquals(new Matrix(3, 3), ma);
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);
        assertEquals(new Matrix(3, 1),
                estimator.getAngularSpeedBias());
        final Matrix bg1 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg1);
        assertEquals(new Matrix(3, 1), bg1);
        assertArrayEquals(new double[3], estimator.getAngularSpeedBiasArray(),
                0.0);
        final double[] bg2 = new double[3];
        estimator.getAngularSpeedBiasArray(bg2);
        assertArrayEquals(new double[3], bg2, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, bgTriad1.getValueX(), 0.0);
        assertEquals(0.0, bgTriad1.getValueY(), 0.0);
        assertEquals(0.0, bgTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgTriad1.getUnit());
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad1, bgTriad2);
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedCrossCouplingErrors());
        final Matrix mg = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg);
        assertEquals(new Matrix(3, 3), mg);
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());
        final Matrix gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
    }

    @Test
    public void testConstructor9() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(
                baTriad, ma, bgTriad, mg, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());
        final Matrix gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(baTriad, wrong, bgTriad, mg,
                    kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(baTriad, ma, bgTriad, wrong,
                    kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(baTriad,
                    new Matrix(1, 3), bgTriad, mg,
                    kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(baTriad,
                    new Matrix(3, 1), bgTriad, mg,
                    kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(baTriad, ma, bgTriad,
                    new Matrix(1, 3), kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(baTriad, ma, bgTriad,
                    new Matrix(3, 1), kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor10() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(
                baTriad, ma, bgTriad, mg, kalmanConfig, initConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());
        final Matrix gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(baTriad, wrong, bgTriad, mg,
                    kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(baTriad, ma, bgTriad, wrong,
                    kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(baTriad,
                    new Matrix(1, 3), bgTriad, mg,
                    kalmanConfig, initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(baTriad,
                    new Matrix(3, 1), bgTriad, mg,
                    kalmanConfig, initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(baTriad, ma, bgTriad,
                    new Matrix(1, 3), kalmanConfig, initConfig,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(baTriad, ma, bgTriad,
                    new Matrix(3, 1), kalmanConfig, initConfig,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor11() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(
                baTriad, ma, bgTriad, mg, gg, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(baTriad, wrong, bgTriad, mg, gg,
                    kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(baTriad, ma, bgTriad, wrong, gg,
                    kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(baTriad,
                    new Matrix(1, 3), bgTriad, mg, gg,
                    kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(baTriad,
                    new Matrix(3, 1), bgTriad, mg, gg,
                    kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(baTriad, ma, bgTriad,
                    new Matrix(1, 3), gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(baTriad, ma, bgTriad,
                    new Matrix(3, 1), gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(baTriad, ma, bgTriad, mg,
                    new Matrix(1, 3), kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(baTriad, ma, bgTriad, mg,
                    new Matrix(3, 1), kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor12() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(
                baTriad, ma, bgTriad, mg, gg, kalmanConfig, initConfig,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(baTriad, wrong, bgTriad, mg, gg,
                    kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(baTriad, ma, bgTriad, wrong, gg,
                    kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(baTriad,
                    new Matrix(1, 3), bgTriad, mg, gg,
                    kalmanConfig, initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(baTriad,
                    new Matrix(3, 1), bgTriad, mg, gg,
                    kalmanConfig, initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(baTriad, ma, bgTriad,
                    new Matrix(1, 3), gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(baTriad, ma, bgTriad,
                    new Matrix(3, 1), gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(baTriad, ma, bgTriad, mg,
                    new Matrix(1, 3), kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(baTriad, ma, bgTriad, mg,
                    new Matrix(3, 1), kalmanConfig, initConfig,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor13() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(ba, ma, bg, mg,
                kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());
        final Matrix gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(ba, wrong, bg, mg,
                    kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma, bg, wrong,
                    kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(new Matrix(1, 1),
                    ma, bg, mg, kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(new Matrix(3, 3),
                    ma, bg, mg, kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba,
                    new Matrix(1, 3), bg, mg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba,
                    new Matrix(3, 1), bg, mg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma,
                    new Matrix(1, 1), mg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma,
                    new Matrix(3, 3), mg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma, bg,
                    new Matrix(1, 3), kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma, bg,
                    new Matrix(3, 1), kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor14() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(ba, ma, bg, mg,
                kalmanConfig, initConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());
        final Matrix gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(ba, wrong, bg, mg, kalmanConfig,
                    initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma, bg, wrong, kalmanConfig,
                    initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(new Matrix(1, 1),
                    ma, bg, mg, kalmanConfig, initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(new Matrix(3, 3),
                    ma, bg, mg, kalmanConfig, initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba,
                    new Matrix(1, 3), bg, mg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba,
                    new Matrix(3, 1), bg, mg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma,
                    new Matrix(1, 1), mg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma,
                    new Matrix(3, 3), mg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma, bg,
                    new Matrix(1, 3), kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma, bg,
                    new Matrix(3, 1), kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor15() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(
                ba, ma, bg, mg, gg, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(ba, wrong, bg, mg, gg,
                    kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma, bg, wrong, gg,
                    kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(new Matrix(1, 1),
                    ma, bg, mg, gg, kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(new Matrix(3, 3),
                    ma, bg, mg, gg, kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, new Matrix(1, 3),
                    bg, mg, gg, kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, new Matrix(3, 1),
                    bg, mg, gg, kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma,
                    new Matrix(1, 1), mg, gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma,
                    new Matrix(3, 3), mg, gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma, bg,
                    new Matrix(1, 3), gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma, bg,
                    new Matrix(3, 1), gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma, bg, mg,
                    new Matrix(1, 3), kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma, bg, mg,
                    new Matrix(3, 1), kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor16() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(
                ba, ma, bg, mg, gg, kalmanConfig, initConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(ba, wrong, bg, mg, gg,
                    kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma, bg, wrong, gg,
                    kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(new Matrix(1, 1),
                    ma, bg, mg, gg, kalmanConfig, initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(new Matrix(3, 3),
                    ma, bg, mg, gg, kalmanConfig, initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba,
                    new Matrix(1, 3), bg, mg, gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba,
                    new Matrix(3, 1), bg, mg, gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma,
                    new Matrix(1, 1), mg, gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma,
                    new Matrix(3, 3), mg, gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma, bg,
                    new Matrix(1, 3), gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma, bg,
                    new Matrix(3, 1), gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma, bg, mg,
                    new Matrix(1, 3), kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ba, ma, bg, mg,
                    new Matrix(3, 1), kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor17() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(ecefFrame,
                baTriad, ma, bgTriad, mg, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final CoordinateTransformation ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final CoordinateTransformation nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());
        final Matrix gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad, wrong,
                    bgTriad, mg, kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad, ma,
                    bgTriad, wrong, kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad,
                    new Matrix(1, 3), bgTriad, mg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad,
                    new Matrix(3, 1), bgTriad, mg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad,
                    new Matrix(1, 3), kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad,
                    new Matrix(3, 1), kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor18() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(ecefFrame,
                baTriad, ma, bgTriad, mg, kalmanConfig, initConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final CoordinateTransformation ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final CoordinateTransformation nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());
        final Matrix gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad, wrong,
                    bgTriad, mg, kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad, ma,
                    bgTriad, wrong, kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad,
                    new Matrix(1, 3), bgTriad, mg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad,
                    new Matrix(3, 1), bgTriad, mg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad,
                    new Matrix(1, 3), kalmanConfig, initConfig,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad,
                    new Matrix(3, 1), kalmanConfig, initConfig,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor19() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(ecefFrame,
                baTriad, ma, bgTriad, mg, gg, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final CoordinateTransformation ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final CoordinateTransformation nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad, wrong,
                    bgTriad, mg, gg, kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad,
                    wrong, gg, kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad,
                    new Matrix(1, 3), bgTriad, mg, gg,
                    kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad,
                    new Matrix(3, 1), bgTriad, mg, gg,
                    kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad,
                    new Matrix(1, 3), gg, kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad,
                    new Matrix(3, 1), gg, kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad,
                    mg, new Matrix(1, 3), kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad,
                    mg, new Matrix(3, 1), kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor20() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(ecefFrame,
                baTriad, ma, bgTriad, mg, gg, kalmanConfig, initConfig,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final CoordinateTransformation ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final CoordinateTransformation nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad, wrong,
                    bgTriad, mg, gg, kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad, ma,
                    bgTriad, wrong, gg, kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad,
                    new Matrix(1, 3), bgTriad, mg, gg,
                    kalmanConfig, initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad,
                    new Matrix(3, 1), bgTriad, mg, gg,
                    kalmanConfig, initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad,
                    new Matrix(1, 3), gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad,
                    new Matrix(3, 1), gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad,
                    mg, new Matrix(1, 3), kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad,
                    mg, new Matrix(3, 1), kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor21() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(ecefFrame,
                ba, ma, bg, mg, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final CoordinateTransformation ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final CoordinateTransformation nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());
        final Matrix gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, wrong, bg, mg,
                    kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma, bg, wrong,
                    kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(ecefFrame,
                    new Matrix(1, 1), ma, bg, mg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame,
                    new Matrix(3, 3), ma, bg, mg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba,
                    new Matrix(1, 3), bg, mg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba,
                    new Matrix(3, 1), bg, mg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma,
                    new Matrix(1, 1), mg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma,
                    new Matrix(3, 3), mg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma, bg,
                    new Matrix(1, 3), kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma, bg,
                    new Matrix(3, 1), kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor22() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(ecefFrame,
                ba, ma, bg, mg, kalmanConfig, initConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final CoordinateTransformation ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final CoordinateTransformation nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());
        final Matrix gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, wrong, bg, mg,
                    kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma, bg, wrong,
                    kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(ecefFrame,
                    new Matrix(1, 1), ma, bg, mg,
                    kalmanConfig, initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame,
                    new Matrix(3, 3), ma, bg, mg,
                    kalmanConfig, initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba,
                    new Matrix(1, 3), bg, mg,
                    kalmanConfig, initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba,
                    new Matrix(3, 1), bg, mg,
                    kalmanConfig, initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma,
                    new Matrix(1, 1), mg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma,
                    new Matrix(3, 3), mg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma, bg,
                    new Matrix(1, 3), kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma, bg,
                    new Matrix(3, 1), kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor23() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(ecefFrame,
                ba, ma, bg, mg, gg, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final CoordinateTransformation ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final CoordinateTransformation nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, wrong, bg, mg, gg,
                    kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma, bg, wrong, gg,
                    kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(ecefFrame,
                    new Matrix(1, 1), ma, bg, mg, gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame,
                    new Matrix(3, 3), ma, bg, mg, gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba,
                    new Matrix(1, 3), bg, mg, gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba,
                    new Matrix(3, 1), bg, mg, gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma,
                    new Matrix(1, 1), mg, gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma,
                    new Matrix(3, 3), mg, gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma, bg,
                    new Matrix(1, 3), gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma, bg,
                    new Matrix(3, 1), gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma, bg, mg,
                    new Matrix(1, 3), kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma, bg, mg,
                    new Matrix(3, 1), kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor24() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(ecefFrame,
                ba, ma, bg, mg, gg, kalmanConfig, initConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final CoordinateTransformation ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final CoordinateTransformation nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, wrong, bg, mg, gg,
                    kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma, bg, wrong, gg,
                    kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(ecefFrame,
                    new Matrix(1, 1), ma, bg, mg, gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame,
                    new Matrix(3, 3), ma, bg, mg, gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba,
                    new Matrix(1, 3), bg, mg, gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba,
                    new Matrix(3, 1), bg, mg, gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma,
                    new Matrix(1, 1), mg, gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma,
                    new Matrix(3, 3), mg, gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma, bg,
                    new Matrix(1, 3), gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma, bg,
                    new Matrix(3, 1), gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma, bg, mg,
                    new Matrix(1, 3), kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(ecefFrame, ba, ma, bg, mg,
                    new Matrix(3, 1), kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor25() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(nedFrame,
                baTriad, ma, bgTriad, mg, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final CoordinateTransformation ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final CoordinateTransformation nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());
        final Matrix gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad, wrong, bgTriad,
                    mg, kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad,
                    wrong, kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad,
                    new Matrix(1, 3), bgTriad, mg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad,
                    new Matrix(3, 1), bgTriad, mg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad,
                    new Matrix(1, 3), kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad,
                    new Matrix(3, 1), kalmanConfig, initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor26() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(nedFrame,
                baTriad, ma, bgTriad, mg, kalmanConfig, initConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final CoordinateTransformation ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final CoordinateTransformation nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());
        final Matrix gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad, wrong,
                    bgTriad, mg, kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad,
                    wrong, kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad,
                    new Matrix(1, 3), bgTriad, mg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad,
                    new Matrix(3, 1), bgTriad, mg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad,
                    new Matrix(1, 3), kalmanConfig, initConfig,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad,
                    new Matrix(3, 1), kalmanConfig, initConfig,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor27() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(nedFrame,
                baTriad, ma, bgTriad, mg, gg, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final CoordinateTransformation ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final CoordinateTransformation nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad, wrong, bgTriad,
                    mg, gg, kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad,
                    wrong, gg, kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad,
                    new Matrix(1, 3), bgTriad, mg, gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad,
                    new Matrix(3, 1), bgTriad, mg, gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad,
                    new Matrix(1, 3), gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad,
                    new Matrix(3, 1), gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad,
                    mg, new Matrix(1, 3), kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad,
                    mg, new Matrix(3, 1), kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor28() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(nedFrame,
                baTriad, ma, bgTriad, mg, gg, kalmanConfig, initConfig,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final CoordinateTransformation ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final CoordinateTransformation nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad, wrong,
                    bgTriad, mg, gg, kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad,
                    wrong, gg, kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad,
                    new Matrix(1, 3), bgTriad, mg, gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad,
                    new Matrix(3, 1), bgTriad, mg, gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad,
                    new Matrix(1, 3), gg, kalmanConfig, initConfig,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad,
                    new Matrix(3, 1), gg, kalmanConfig, initConfig,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad,
                    mg, new Matrix(1, 3), kalmanConfig, initConfig,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad,
                    mg, new Matrix(3, 1), kalmanConfig, initConfig,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor29() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(nedFrame,
                ba, ma, bg, mg, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final CoordinateTransformation ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final CoordinateTransformation nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());
        final Matrix gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, wrong, bg, mg,
                    kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg, wrong,
                    kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(nedFrame,
                    new Matrix(1, 1), ma, bg, mg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame,
                    new Matrix(3, 3), ma, bg, mg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba,
                    new Matrix(1, 3), bg, mg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba,
                    new Matrix(3, 1), bg, mg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma,
                    new Matrix(1, 1), mg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma,
                    new Matrix(3, 3), mg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg,
                    new Matrix(1, 3), kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg,
                    new Matrix(3, 1), kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor30() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(nedFrame,
                ba, ma, bg, mg, kalmanConfig, initConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final CoordinateTransformation ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final CoordinateTransformation nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());
        final Matrix gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, wrong, bg, mg,
                    kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg, wrong,
                    kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(nedFrame,
                    new Matrix(1, 1), ma, bg, mg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame,
                    new Matrix(3, 3), ma, bg, mg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba,
                    new Matrix(1, 3), bg, mg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba,
                    new Matrix(3, 1), bg, mg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma,
                    new Matrix(1, 1), mg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma,
                    new Matrix(3, 3), mg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg,
                    new Matrix(1, 3), kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg,
                    new Matrix(3, 1), kalmanConfig, initConfig,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor31() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(nedFrame,
                ba, ma, bg, mg, gg, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final CoordinateTransformation ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final CoordinateTransformation nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, wrong, bg, mg, gg,
                    kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg, wrong, gg,
                    kalmanConfig, initConfig);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(nedFrame,
                    new Matrix(1, 1), ma, bg, mg, gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame,
                    new Matrix(3, 3), ma, bg, mg, gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba,
                    new Matrix(1, 3), bg, mg, gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba,
                    new Matrix(3, 1), bg, mg, gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma,
                    new Matrix(1, 1), mg, gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma,
                    new Matrix(3, 3), mg, gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg,
                    new Matrix(1, 3), gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg,
                    new Matrix(3, 1), gg, kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg, mg,
                    new Matrix(1, 3), kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg, mg,
                    new Matrix(3, 1), kalmanConfig,
                    initConfig);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor32() throws AlgebraException {
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();

        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final Matrix ba = generateBa();
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

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

        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final double gsx = mg.getElementAt(0, 0);
        final double gsy = mg.getElementAt(1, 1);
        final double gsz = mg.getElementAt(2, 2);
        final double gmxy = mg.getElementAt(0, 1);
        final double gmxz = mg.getElementAt(0, 2);
        final double gmyx = mg.getElementAt(1, 0);
        final double gmyz = mg.getElementAt(1, 2);
        final double gmzx = mg.getElementAt(2, 0);
        final double gmzy = mg.getElementAt(2, 1);

        final AccelerationTriad baTriad = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final AngularSpeedTriad bgTriad = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(nedFrame,
                ba, ma, bg, mg, gg, kalmanConfig, initConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final CoordinateTransformation ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final CoordinateTransformation nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final Matrix ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final double[] ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final double[] ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final AccelerationTriad baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final AccelerationTriad baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final Acceleration baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final Acceleration baX2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final Acceleration baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final Acceleration baY2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final Acceleration baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final Acceleration baZ2 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final AngularSpeedTriad bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final AngularSpeedTriad bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final AngularSpeed bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final AngularSpeed bgX2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final AngularSpeed bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final AngularSpeed bgY2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final AngularSpeed bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final AngularSpeed bgZ2 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3)
                .multiplyByScalarAndReturnNew(-1.0);

        estimator = null;
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, wrong, bg, mg, gg,
                    kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg, wrong, gg,
                    kalmanConfig, initConfig, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        assertNull(estimator);

        // Force IllegalArgumentException
        try {
            estimator = new KalmanDriftEstimator(nedFrame,
                    new Matrix(1, 1), ma, bg, mg, gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame,
                    new Matrix(3, 3), ma, bg, mg, gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba,
                    new Matrix(1, 3), bg, mg, gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba,
                    new Matrix(3, 1), bg, mg, gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma,
                    new Matrix(1, 1), mg, gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma,
                    new Matrix(3, 3), mg, gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg,
                    new Matrix(1, 3), gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg,
                    new Matrix(3, 1), gg, kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg, mg,
                    new Matrix(1, 3), kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg, mg,
                    new Matrix(3, 1), kalmanConfig,
                    initConfig, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testGetSetReferenceFrame() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getReferenceFrame());

        // set new value
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        estimator.setReferenceFrame(ecefFrame);

        // check
        assertSame(ecefFrame, estimator.getReferenceFrame());
        assertTrue(nedFrame.equals(estimator.getReferenceNedFrame(),
                FRAME_ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetReferenceNedFrame() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getReferenceNedFrame());

        // set new value
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        estimator.setReferenceNedFrame(nedFrame);

        // check
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);

        // set again
        estimator.setReferenceNedFrame(nedFrame);

        // check
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final NEDFrame nedFrame3 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame3, FRAME_ABSOLUTE_ERROR));
        final NEDFrame nedFrame4 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame4));
        assertEquals(nedFrame3, nedFrame4);

        // set to null
        estimator.setReferenceNedFrame(null);

        // check
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
    }

    @Test
    public void testGetSetReferenceEcefPosition() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));

        // set new value
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();
        final NEDPosition nedPosition = nedFrame.getPosition();

        estimator.setReferenceEcefPosition(ecefPosition);

        // check
        final ECEFPosition ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefPosition, ecefPosition1);
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition.equals(nedPosition1, FRAME_ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetReferenceEcefVelocity() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));

        // set a random position to avoid singularity at Earth's center
        final NEDFrame nedFrame = new NEDFrame();
        nedFrame.setVelocityCoordinates(1.0, 2.0, 3.0);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();
        estimator.setReferenceEcefPosition(ecefPosition);

        // set new velocity value
        final ECEFVelocity ecefVelocity = ecefFrame.getECEFVelocity();
        final NEDVelocity nedVelocity = nedFrame.getVelocity();

        estimator.setReferenceEcefVelocity(ecefVelocity);

        // check
        final ECEFVelocity ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefVelocity, ecefVelocity1);
        final ECEFVelocity ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity, ecefVelocity2);
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity.equals(nedVelocity1, FRAME_ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetReferenceEcefCoordinateTransformation()
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));

        // set new value
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final CoordinateTransformation ecefC = ecefFrame
                .getCoordinateTransformation();
        final CoordinateTransformation nedC = nedFrame
                .getCoordinateTransformation();

        estimator.setReferenceEcefCoordinateTransformation(ecefC);

        // check
        final CoordinateTransformation ecefC1 = estimator
                .getReferenceEcefCoordinateTransformation();
        assertEquals(ecefC, ecefC1);
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getReferenceEcefCoordinateTransformation(ecefC2);
        assertEquals(ecefC, ecefC2);
        assertTrue(nedC.equals(estimator.getReferenceNedCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));

        // set again
        estimator.setReferenceEcefCoordinateTransformation(ecefC);

        // check
        final CoordinateTransformation ecefC3 = estimator
                .getReferenceEcefCoordinateTransformation();
        assertEquals(ecefC, ecefC3);
        final CoordinateTransformation ecefC4 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getReferenceEcefCoordinateTransformation(ecefC4);
        assertEquals(ecefC, ecefC4);
        assertTrue(nedC.equals(estimator.getReferenceNedCoordinateTransformation(),
                FRAME_ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetReferenceNedPosition() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));

        // set new value
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final NEDPosition nedPosition = nedFrame.getPosition();
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        estimator.setReferenceNedPosition(nedPosition);

        // check
        final NEDPosition nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition.equals(nedPosition1, FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        assertEquals(ecefPosition, estimator.getReferenceEcefPosition());

        // set again
        estimator.setReferenceNedPosition(nedPosition);

        // check
        final NEDPosition nedPosition3 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition.equals(nedPosition3, FRAME_ABSOLUTE_ERROR));
        final NEDPosition nedPosition4 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition4));
        assertEquals(nedPosition3, nedPosition4);
        assertEquals(ecefPosition, estimator.getReferenceEcefPosition());
    }

    @Test
    public void testGetSetReferenceNedVelocity() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));

        // set new value
        final NEDFrame nedFrame = new NEDFrame();
        nedFrame.setVelocityCoordinates(1.0, 2.0, 3.0);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final NEDVelocity nedVelocity = nedFrame.getVelocity();
        final ECEFVelocity ecefVelocity = ecefFrame.getECEFVelocity();

        estimator.setReferenceNedVelocity(nedVelocity);

        // check
        final NEDVelocity nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity.equals(nedVelocity1, FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        assertEquals(ecefVelocity, estimator.getReferenceEcefVelocity());

        // set again
        estimator.setReferenceNedVelocity(nedVelocity);

        // check
        final NEDVelocity nedVelocity3 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity.equals(nedVelocity3, FRAME_ABSOLUTE_ERROR));
        final NEDVelocity nedVelocity4 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity4));
        assertEquals(nedVelocity3, nedVelocity4);
        assertEquals(ecefVelocity, estimator.getReferenceEcefVelocity());
    }

    @Test
    public void testGetSetReferenceNedCoordinateTransformation()
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));

        // set new value
        // set new value
        final NEDFrame nedFrame = new NEDFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final CoordinateTransformation ecefC = ecefFrame
                .getCoordinateTransformation();
        final CoordinateTransformation nedC = nedFrame
                .getCoordinateTransformation();

        estimator.setReferenceNedCoordinateTransformation(nedC);

        // check
        final CoordinateTransformation nedC1 = estimator
                .getReferenceNedCoordinateTransformation();
        assertTrue(nedC.equals(nedC1, FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ecefC, estimator.getReferenceEcefCoordinateTransformation());

        // set again
        estimator.setReferenceNedCoordinateTransformation(nedC);

        // check
        final CoordinateTransformation nedC3 = estimator
                .getReferenceNedCoordinateTransformation();
        assertTrue(nedC.equals(nedC3, FRAME_ABSOLUTE_ERROR));
        final CoordinateTransformation nedC4 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC4));
        assertEquals(nedC3, nedC4);
        assertEquals(ecefC, estimator.getReferenceEcefCoordinateTransformation());
    }

    @Test
    public void testGetSetAccelerationBias() throws WrongSizeException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(new Matrix(3, 1), estimator.getAccelerationBias());

        // set new value
        final Matrix ba = generateBa();
        estimator.setAccelerationBias(ba);

        // check
        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba, ba2);

        // Force IllegalArgumentException
        try {
            estimator.setAccelerationBias(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setAccelerationBias(new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAccelerationBiasArray() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        final double[] ba1 = estimator.getAccelerationBiasArray();
        assertArrayEquals(new double[3], ba1, 0.0);
        final double[] ba2 = new double[3];
        estimator.getAccelerationBiasArray(ba2);
        assertArrayEquals(ba1, ba2, 0.0);

        // set new value
        final double[] ba = generateBa().getBuffer();
        estimator.setAccelerationBias(ba);

        // check
        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba, ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba, ba4, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setAccelerationBias(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAccelerationBiasAsTriad() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());

        // set new value
        final Matrix ba = generateBa();
        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);
        final AccelerationTriad triad2 = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);
        estimator.setAccelerationBias(triad2);

        // check
        final AccelerationTriad triad3 = estimator.getAccelerationBiasAsTriad();
        final AccelerationTriad triad4 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad4);
        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    public void testGetSetAccelerationBiasX() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double bax = ba.getElementAtIndex(0);
        estimator.setAccelerationBiasX(bax);

        // check
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
    }

    @Test
    public void testGetSetAccelerationBiasY() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double bay = ba.getElementAtIndex(1);
        estimator.setAccelerationBiasY(bay);

        // check
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
    }

    @Test
    public void testGetSetAccelerationBiasZ() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double baz = ba.getElementAtIndex(2);
        estimator.setAccelerationBiasZ(baz);

        // check
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
    }

    @Test
    public void testSetAccelerationBias1() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default values
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        // set new values
        final Matrix ba = generateBa();
        final double bax = ba.getElementAtIndex(0);
        final double bay = ba.getElementAtIndex(1);
        final double baz = ba.getElementAtIndex(2);

        estimator.setAccelerationBias(bax, bay, baz);

        // check
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
    }

    @Test
    public void testGetSetAccelerationBiasXAsAcceleration()
            throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, bax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());

        // set new value
        final Matrix ba = generateBa();
        final double bax = ba.getElementAtIndex(0);
        final Acceleration bax2 = new Acceleration(bax,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        estimator.setAccelerationBiasX(bax2);

        // check
        final Acceleration bax3 = estimator.getAccelerationBiasXAsAcceleration();
        final Acceleration bax4 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax4);
        assertEquals(bax2, bax3);
        assertEquals(bax2, bax4);
    }

    @Test
    public void testGetSetAccelerationBiasYAsAcceleration()
            throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, bay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());

        // set new value
        final Matrix ba = generateBa();
        final double bay = ba.getElementAtIndex(1);
        final Acceleration bay2 = new Acceleration(bay,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        estimator.setAccelerationBiasY(bay2);

        // check
        final Acceleration bay3 = estimator.getAccelerationBiasYAsAcceleration();
        final Acceleration bay4 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay4);
        assertEquals(bay2, bay3);
        assertEquals(bay2, bay4);
    }

    @Test
    public void testGetSetAccelerationBiasZAsAcceleration()
            throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());

        // set new value
        final Matrix ba = generateBa();
        final double baz = ba.getElementAtIndex(2);
        final Acceleration baz2 = new Acceleration(baz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);

        estimator.setAccelerationBiasZ(baz2);

        // check
        final Acceleration baz3 = estimator.getAccelerationBiasZAsAcceleration();
        final Acceleration baz4 = new Acceleration(1.0,
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz4);
        assertEquals(baz2, baz3);
        assertEquals(baz2, baz4);
    }

    @Test
    public void testSetAccelerationBias2() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default values
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        // set new values
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

        estimator.setAccelerationBias(bax1, bay1, baz1);

        // check
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
    }

    @Test
    public void testGetSetAccelerationCrossCouplingErrors()
            throws AlgebraException, LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(new Matrix(3, 3),
                estimator.getAccelerationCrossCouplingErrors());

        // set new value
        final Matrix ma = generateMaGeneral();
        estimator.setAccelerationCrossCouplingErrors(ma);

        // check
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma1);
        assertEquals(ma, ma2);

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        try {
            estimator.setAccelerationCrossCouplingErrors(wrong);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator.setAccelerationCrossCouplingErrors(
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setAccelerationCrossCouplingErrors(
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAccelerationSx() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double sx = ma.getElementAt(0, 0);
        estimator.setAccelerationSx(sx);

        // check
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
    }

    @Test
    public void testGetSetAccelerationSy() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double sy = ma.getElementAt(1, 1);
        estimator.setAccelerationSy(sy);

        // check
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
    }

    @Test
    public void testGetSetAccelerationSz() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double sz = ma.getElementAt(2, 2);
        estimator.setAccelerationSz(sz);

        // check
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
    }

    @Test
    public void testGetSetAccelerationMxy() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double mxy = ma.getElementAt(0, 1);
        estimator.setAccelerationMxy(mxy);

        // check
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
    }

    @Test
    public void testGetSetAccelerationMxz() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double mxz = ma.getElementAt(0, 2);
        estimator.setAccelerationMxz(mxz);

        // check
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
    }

    @Test
    public void testGetSetAccelerationMyx() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double myx = ma.getElementAt(1, 0);
        estimator.setAccelerationMyx(myx);

        // check
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
    }

    @Test
    public void testGetSetAccelerationMyz() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double myz = ma.getElementAt(1, 2);
        estimator.setAccelerationMyz(myz);

        // check
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
    }

    @Test
    public void testGetSetAccelerationMzx() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double mzx = ma.getElementAt(2, 0);
        estimator.setAccelerationMzx(mzx);

        // check
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
    }

    @Test
    public void testGetSetAccelerationMzy() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);

        // set new value
        final Matrix ma = generateMaGeneral();
        final double mzy = ma.getElementAt(2, 1);
        estimator.setAccelerationMzy(mzy);

        // check
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);
    }

    @Test
    public void testSetAccelerationScalingFactors() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default values
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);

        // set new values
        final Matrix ma = generateMaGeneral();
        final double sx = ma.getElementAt(0, 0);
        final double sy = ma.getElementAt(1, 1);
        final double sz = ma.getElementAt(2, 2);
        estimator.setAccelerationScalingFactors(sx, sy, sz);

        // check
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
    }

    @Test
    public void testSetAccelerationCrossCouplingErrors() throws AlgebraException, LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default values
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);

        // set new values
        final Matrix ma = generateMaGeneral();
        final double mxy = ma.getElementAt(0, 1);
        final double mxz = ma.getElementAt(0, 2);
        final double myx = ma.getElementAt(1, 0);
        final double myz = ma.getElementAt(1, 2);
        final double mzx = ma.getElementAt(2, 0);
        final double mzy = ma.getElementAt(2, 1);
        estimator.setAccelerationCrossCouplingErrors(
                mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);
    }

    @Test
    public void testSetAccelerationScalingFactorsAndCrossCouplingErrors() throws AlgebraException, LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default values
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);

        // set new values
        final Matrix ma = generateMaGeneral();
        final double sx = ma.getElementAt(0, 0);
        final double sy = ma.getElementAt(1, 1);
        final double sz = ma.getElementAt(2, 2);
        final double mxy = ma.getElementAt(0, 1);
        final double mxz = ma.getElementAt(0, 2);
        final double myx = ma.getElementAt(1, 0);
        final double myz = ma.getElementAt(1, 2);
        final double mzx = ma.getElementAt(2, 0);
        final double mzy = ma.getElementAt(2, 1);
        estimator.setAccelerationScalingFactorsAndCrossCouplingErrors(
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedBias() throws WrongSizeException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(new Matrix(3, 1), estimator.getAngularSpeedBias());

        // set new value
        final Matrix bg = generateBg();
        estimator.setAngularSpeedBias(bg);

        // check
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);

        // Force IllegalArgumentException
        try {
            estimator.setAngularSpeedBias(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setAngularSpeedBias(new Matrix(3, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAngularSpeedBiasArray() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        final double[] bg1 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(new double[3], bg1, 0.0);
        final double[] bg2 = new double[3];
        estimator.getAngularSpeedBiasArray(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        // set new value
        final double[] bg = generateBg().getBuffer();
        estimator.setAngularSpeedBias(bg);

        // check
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg, bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg, bg4, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setAngularSpeedBias(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAngularSpeedBiasAsTriad() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        final AngularSpeedTriad triad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());

        // set new value
        final Matrix bg = generateBg();
        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final AngularSpeedTriad triad2 = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);
        estimator.setAngularSpeedBias(triad2);

        // check
        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    public void testGetSetAngularSpeedBiasX() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double bgx = bg.getElementAtIndex(0);
        estimator.setAngularSpeedBiasX(bgx);

        // check
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedBiasY() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double bgy = bg.getElementAtIndex(1);
        estimator.setAngularSpeedBiasY(bgy);

        // check
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedBiasZ() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double bgz = bg.getElementAtIndex(2);
        estimator.setAngularSpeedBiasZ(bgz);

        // check
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
    }

    @Test
    public void testSetAngularSpeedBias1() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default values
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);

        // set new values
        final Matrix bg = generateBg();
        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        estimator.setAngularSpeedBias(bgx, bgy, bgz);

        // check
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedBiasXAsAngularSpeed()
            throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default values
        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());

        // set new value
        final Matrix bg = generateBg();
        final double bgx = bg.getElementAtIndex(0);
        final AngularSpeed bgx2 = new AngularSpeed(bgx,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        estimator.setAngularSpeedBiasX(bgx2);

        // check
        final AngularSpeed bgx3 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        final AngularSpeed bgx4 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx4);
        assertEquals(bgx2, bgx3);
        assertEquals(bgx2, bgx4);
    }

    @Test
    public void testGetSetAngularSpeedBiasYAsAngularSpeed()
            throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());

        // set new value
        final Matrix bg = generateBg();
        final double bgy = bg.getElementAtIndex(1);
        final AngularSpeed bgy2 = new AngularSpeed(bgy,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        estimator.setAngularSpeedBiasY(bgy2);

        // check
        final AngularSpeed bgy3 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        final AngularSpeed bgy4 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy4);
        assertEquals(bgy2, bgy3);
        assertEquals(bgy2, bgy4);
    }

    @Test
    public void testGetSetAngularSpeedBiasZAsAngularSpeed()
            throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());

        // set new value
        final Matrix bg = generateBg();
        final double bgz = bg.getElementAtIndex(2);
        final AngularSpeed bgz2 = new AngularSpeed(bgz,
                AngularSpeedUnit.RADIANS_PER_SECOND);

        estimator.setAngularSpeedBiasZ(bgz2);

        // check
        final AngularSpeed bgz3 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        final AngularSpeed bgz4 = new AngularSpeed(1.0,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz4);
        assertEquals(bgz2, bgz3);
        assertEquals(bgz2, bgz4);
    }

    @Test
    public void testSetAngularSpeedBias2() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);

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

        estimator.setAngularSpeedBias(bgx1, bgy1, bgz1);

        // check
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedCrossCouplingErrors()
            throws AlgebraException, LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedCrossCouplingErrors());

        // set new value
        final Matrix mg = generateMg();
        estimator.setAngularSpeedCrossCouplingErrors(mg);

        // check
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg1);
        assertEquals(mg, mg2);

        // Force AlgebraException
        final Matrix wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        try {
            estimator.setAngularSpeedCrossCouplingErrors(wrong);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator.setAngularSpeedCrossCouplingErrors(
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setAngularSpeedCrossCouplingErrors(
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAngularSpeedSx() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);

        // set new value
        final Matrix mg = generateMg();
        final double sx = mg.getElementAt(0, 0);
        estimator.setAngularSpeedSx(sx);

        // check
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedSy() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);

        // set new value
        final Matrix mg = generateMg();
        final double sy = mg.getElementAt(1, 1);
        estimator.setAngularSpeedSy(sy);

        // check
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedSz() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);

        // set new value
        final Matrix mg = generateMg();
        final double sz = mg.getElementAt(2, 2);
        estimator.setAngularSpeedSz(sz);

        // check
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedMxy() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);

        // set new value
        final Matrix mg = generateMg();
        final double mxy = mg.getElementAt(0, 1);
        estimator.setAngularSpeedMxy(mxy);

        // check
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedMxz() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);

        // set new value
        final Matrix mg = generateMg();
        final double mxz = mg.getElementAt(0, 2);
        estimator.setAngularSpeedMxz(mxz);

        // check
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedMyx() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);

        // set new value
        final Matrix mg = generateMg();
        final double myx = mg.getElementAt(1, 0);
        estimator.setAngularSpeedMyx(myx);

        // check
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedMyz() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);

        // set new value
        final Matrix mg = generateMg();
        final double myz = mg.getElementAt(1, 2);
        estimator.setAngularSpeedMyz(myz);

        // check
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedMzx() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);

        // set new value
        final Matrix mg = generateMg();
        final double mzx = mg.getElementAt(2, 0);
        estimator.setAngularSpeedMzx(mzx);

        // check
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedMzy() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);

        // set new value
        final Matrix mg = generateMg();
        final double mzy = mg.getElementAt(2, 1);
        estimator.setAngularSpeedMzy(mzy);

        // check
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);
    }

    @Test
    public void testSetAngularSpeedScalingFactors() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default values
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);

        // set new values
        final Matrix mg = generateMg();
        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        estimator.setAngularSpeedScalingFactors(sx, sy, sz);

        // check
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
    }

    @Test
    public void testSetAngularSpeedCrossCouplingErrors() throws AlgebraException,
            LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default values
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);

        // set new values
        final Matrix mg = generateMg();
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);
        estimator.setAngularSpeedCrossCouplingErrors(
                mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);
    }

    @Test
    public void testSetAngularSpeedScalingFactorAndCrossCouplingErrors()
            throws AlgebraException, LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default values
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);

        // set new values
        final Matrix mg = generateMg();
        final double sx = mg.getElementAt(0, 0);
        final double sy = mg.getElementAt(1, 1);
        final double sz = mg.getElementAt(2, 2);
        final double mxy = mg.getElementAt(0, 1);
        final double mxz = mg.getElementAt(0, 2);
        final double myx = mg.getElementAt(1, 0);
        final double myz = mg.getElementAt(1, 2);
        final double mzx = mg.getElementAt(2, 0);
        final double mzy = mg.getElementAt(2, 1);
        estimator.setAngularSpeedScalingFactorsAndCrossCouplingErrors(
                sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);
    }

    @Test
    public void testGetSetAngularSpeedGDependentCrossBias()
            throws WrongSizeException, LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(new Matrix(3, 3),
                estimator.getAngularSpeedGDependantCrossBias());

        // set new value
        final Matrix gg = generateGg();
        estimator.setAngularSpeedGDependantCrossBias(gg);

        // check
        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg1);
        assertEquals(gg, gg2);

        // Force IllegalArgumentException
        try {
            estimator.setAngularSpeedGDependantCrossBias(
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setAngularSpeedGDependantCrossBias(
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testIsSetFixKinematicsEnabled() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        assertTrue(estimator.isFixKinematicsEnabled());

        // set new value
        estimator.setFixKinematicsEnabled(false);

        // check
        assertFalse(estimator.isFixKinematicsEnabled());
    }

    @Test
    public void testGetSetTimeInterval() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        // set new value
        estimator.setTimeInterval(1.0);

        // check
        assertEquals(1.0, estimator.getTimeInterval(), 0.0);

        // Force IllegalArgumentException
        try {
            estimator.setTimeInterval(-1.0);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetTimeIntervalAsTime() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        final Time timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());

        // set new value
        final Time timeInterval2 = new Time(1.0, TimeUnit.SECOND);
        estimator.setTimeInterval(timeInterval2);

        // check
        final Time timeInterval3 = estimator.getTimeIntervalAsTime();
        final Time timeInterval4 = new Time(3.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval4);
        assertEquals(timeInterval2, timeInterval3);
        assertEquals(timeInterval2, timeInterval4);
    }

    @Test
    public void testGetSetKalmanConfig() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getKalmanConfig());

        // set new value
        final INSLooselyCoupledKalmanConfig kalmanConfig =
                new INSLooselyCoupledKalmanConfig();
        estimator.setKalmanConfig(kalmanConfig);

        // check
        assertSame(kalmanConfig, estimator.getKalmanConfig());
    }

    @Test
    public void testGetSetInitConfig() throws LockedException {
        final KalmanDriftEstimator estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getInitConfig());

        // set new value
        final INSLooselyCoupledKalmanInitializerConfig initConfig =
                new INSLooselyCoupledKalmanInitializerConfig();
        estimator.setInitConfig(initConfig);

        // check
        assertSame(initConfig, estimator.getInitConfig());
    }

    @Test
    public void testAddBodyKinematicsAndResetExactCalibrationNoNoiseAndKinematicsFixed()
            throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            InertialNavigatorException, DriftEstimationException,
            NotReadyException, InvalidRotationMatrixException,
            RotationException, INSException {

        final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig = generateInitConfig();

        final Matrix ba = generateBa();
        final Matrix ma = generateMaCommonAxis();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        final NEDFrame nedFrame = generateFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final Random random = new Random();

        final CoordinateTransformation ecefC = ecefFrame
                .getCoordinateTransformation();
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final ECEFFrame navigationFrame = new ECEFFrame(ecefFrame);
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();
        fixer.setAccelerationBias(ba);
        fixer.setAccelerationCrossCouplingErrors(ma);
        fixer.setAngularSpeedBias(bg);
        fixer.setAngularSpeedCrossCouplingErrors(mg);
        fixer.setAngularSpeedGDependantCrossBias(gg);

        final KalmanDriftEstimator kalmanDriftEstimator = new KalmanDriftEstimator(
                nedFrame, ba, ma, bg, mg, gg, kalmanConfig, initConfig, this);
        kalmanDriftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

        final DriftEstimator driftEstimator = new DriftEstimator(nedFrame,
                ba, ma, bg, mg, gg);
        driftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

        final INSLooselyCoupledKalmanFilteredEstimator kalmanEstimator =
                new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                        initConfig, new ECEFFrame(ecefFrame));

        reset();
        assertEquals(mStart, 0);
        assertEquals(mBodyKinematicsAdded, 0);
        assertEquals(mReset, 0);
        assertEquals(kalmanDriftEstimator.getNumberOfProcessedSamples(), 0);
        assertFalse(kalmanDriftEstimator.isRunning());
        assertTrue(kalmanDriftEstimator.isFixKinematicsEnabled());

        final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                        ecefC, ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition);
        final BodyKinematics fixedKinematics = new BodyKinematics();

        final BodyKinematics measuredKinematics = new BodyKinematics();
        for (int i = 0; i < N_SAMPLES; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);

            kalmanDriftEstimator.addBodyKinematics(measuredKinematics);
            driftEstimator.addBodyKinematics(measuredKinematics);

            assertEquals(kalmanDriftEstimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(kalmanDriftEstimator.isRunning());

            fixer.fix(measuredKinematics, fixedKinematics);
            ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                    navigationFrame, fixedKinematics, navigationFrame);

            kalmanEstimator.update(fixedKinematics, i * TIME_INTERVAL_SECONDS);
        }

        assertEquals(kalmanDriftEstimator.getNumberOfProcessedSamples(), N_SAMPLES);
        assertFalse(kalmanDriftEstimator.isRunning());
        assertEquals(mStart, 1);
        assertEquals(mBodyKinematicsAdded, N_SAMPLES);
        assertEquals(mReset, 0);

        final INSLooselyCoupledKalmanState kalmanState = kalmanEstimator.getState();

        final double navigationPositionDrift = positionDrift(
                ecefFrame, navigationFrame);
        final double navigationVelocityDrift = velocityDrift(
                ecefFrame, navigationFrame);
        final double navigationOrientationDrift = orientationDrift(
                ecefFrame, navigationFrame);

        final double kalmanPositionDrift = positionDrift(
                ecefFrame, kalmanState);
        final double kalmanVelocityDrift = velocityDrift(
                ecefFrame, kalmanState);
        final double kalmanOrientationDrift = orientationDrift(
                ecefFrame, kalmanState);

        final ECEFPosition currentPositionDrift1 = kalmanDriftEstimator
                .getCurrentPositionDrift();
        final ECEFPosition currentPositionDrift2 = new ECEFPosition();
        assertTrue(kalmanDriftEstimator.getCurrentPositionDrift(currentPositionDrift2));
        assertEquals(currentPositionDrift1, currentPositionDrift2);

        final ECEFVelocity currentVelocityDrift1 = kalmanDriftEstimator
                .getCurrentVelocityDrift();
        final ECEFVelocity currentVelocityDrift2 = new ECEFVelocity();
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDrift(currentVelocityDrift2));
        assertEquals(currentVelocityDrift1, currentVelocityDrift2);

        final Rotation3D currentOrientationDrift1 = kalmanDriftEstimator
                .getCurrentOrientationDrift();
        final Quaternion currentOrientationDrift2 = new Quaternion();
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDrift(currentOrientationDrift2));
        assertEquals(currentOrientationDrift1, currentOrientationDrift2);

        final DistanceFormatter distanceFormatter = new DistanceFormatter();
        final SpeedFormatter speedFormatter = new SpeedFormatter();
        final AngleFormatter angleFormatter = new AngleFormatter();
        final AccelerationFormatter accelerationFormatter =
                new AccelerationFormatter();
        final AngularSpeedFormatter angularSpeedFormatter =
                new AngularSpeedFormatter();

        final Double currentPositionDriftNorm = kalmanDriftEstimator
                .getCurrentPositionDriftNormMeters();
        assertNotNull(currentPositionDriftNorm);
        assertEquals(currentPositionDriftNorm, currentPositionDrift1.getNorm(),
                0.0);
        LOGGER.log(Level.INFO, "Current position drift: " +
                distanceFormatter.format(currentPositionDriftNorm,
                        DistanceUnit.METER));
        assertTrue(currentPositionDriftNorm < 8.0);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentPositionDriftNorm < navigationPositionDrift);
        assertEquals(currentPositionDriftNorm, kalmanPositionDrift,
                ABSOLUTE_ERROR);

        final Distance currentPositionDriftNorm1 = kalmanDriftEstimator
                .getCurrentPositionDriftNorm();
        assertEquals(currentPositionDriftNorm,
                currentPositionDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, currentPositionDriftNorm1.getUnit());
        final Distance currentPositionDriftNorm2 = new Distance(1.0,
                DistanceUnit.FOOT);
        assertTrue(kalmanDriftEstimator.getCurrentPositionDriftNorm(
                currentPositionDriftNorm2));
        assertEquals(currentPositionDriftNorm1, currentPositionDriftNorm2);

        final Double currentVelocityDriftNorm = kalmanDriftEstimator
                .getCurrentVelocityDriftNormMetersPerSecond();
        assertNotNull(currentVelocityDriftNorm);
        assertEquals(currentVelocityDriftNorm, currentVelocityDrift1.getNorm(),
                0.0);
        LOGGER.log(Level.INFO, "Current velocity drift: " +
                speedFormatter.format(currentVelocityDriftNorm,
                        SpeedUnit.METERS_PER_SECOND));
        assertTrue(currentVelocityDriftNorm < 0.7);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentVelocityDriftNorm < navigationVelocityDrift);
        assertEquals(currentVelocityDriftNorm, kalmanVelocityDrift,
                ABSOLUTE_ERROR);

        final Speed currentVelocityDriftNorm1 = kalmanDriftEstimator
                .getCurrentVelocityDriftNorm();
        assertEquals(currentVelocityDriftNorm,
                currentVelocityDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND,
                currentVelocityDriftNorm1.getUnit());
        final Speed currentVelocityDriftNorm2 = new Speed(1.0,
                SpeedUnit.KILOMETERS_PER_HOUR);
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftNorm(
                currentVelocityDriftNorm2));
        assertEquals(currentVelocityDriftNorm1, currentVelocityDriftNorm2);

        final Double currentOrientationDriftNorm = kalmanDriftEstimator
                .getCurrentOrientationDriftRadians();
        assertNotNull(currentOrientationDriftNorm);
        assertEquals(currentOrientationDriftNorm,
                currentOrientationDrift1.getRotationAngle(), 0.0);
        LOGGER.log(Level.INFO, "Current orientation drift: " +
                angleFormatter.format(AngleConverter.convert(
                        currentOrientationDriftNorm, AngleUnit.RADIANS,
                        AngleUnit.DEGREES), AngleUnit.DEGREES));
        assertTrue(currentOrientationDriftNorm < 0.006);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentOrientationDriftNorm < navigationOrientationDrift);
        assertEquals(currentOrientationDriftNorm, kalmanOrientationDrift,
                ABSOLUTE_ERROR);

        final Angle currentOrientationDriftNorm1 = kalmanDriftEstimator
                .getCurrentOrientationDriftAngle();
        assertEquals(currentOrientationDriftNorm,
                currentOrientationDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, currentOrientationDriftNorm1.getUnit());
        final Angle currentOrientationDriftNorm2 = new Angle(1.0,
                AngleUnit.DEGREES);
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftAngle(
                currentOrientationDriftNorm2));
        assertEquals(currentOrientationDriftNorm1, currentOrientationDriftNorm2);

        final Double currentPositionDriftPerTimeUnit =
                kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit();
        assertNotNull(currentPositionDriftPerTimeUnit);
        LOGGER.log(Level.INFO, "Current position drift per time unit: " +
                speedFormatter.format(currentPositionDriftPerTimeUnit,
                        SpeedUnit.METERS_PER_SECOND));
        assertTrue(currentPositionDriftPerTimeUnit < 0.3);
        assertEquals(currentPositionDriftPerTimeUnit,
                currentPositionDriftNorm / (N_SAMPLES * TIME_INTERVAL_SECONDS),
                ABSOLUTE_ERROR);

        final Speed currentPositionDriftPerTimeUnit1 =
                kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed();
        assertEquals(currentPositionDriftPerTimeUnit,
                currentPositionDriftPerTimeUnit1.getValue().doubleValue(),
                0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND,
                currentPositionDriftPerTimeUnit1.getUnit());
        final Speed currentPositionDriftPerTimeUnit2 = new Speed(1.0,
                SpeedUnit.KILOMETERS_PER_HOUR);
        assertTrue(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(
                currentPositionDriftPerTimeUnit2));
        assertEquals(currentPositionDriftPerTimeUnit1,
                currentPositionDriftPerTimeUnit2);

        final Double currentVelocityDriftPerTimeUnit =
                kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit();
        assertNotNull(currentVelocityDriftPerTimeUnit);
        LOGGER.log(Level.INFO, "Current velocity drift per time unit: " +
                accelerationFormatter.format(currentVelocityDriftPerTimeUnit,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertTrue(currentVelocityDriftPerTimeUnit < 0.03);
        assertEquals(currentVelocityDriftPerTimeUnit,
                currentVelocityDriftNorm / (N_SAMPLES * TIME_INTERVAL_SECONDS),
                ABSOLUTE_ERROR);

        final Acceleration currentVelocityDriftPerTimeUnit1 =
                kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration();
        assertEquals(currentVelocityDriftPerTimeUnit,
                currentVelocityDriftPerTimeUnit1.getValue().doubleValue(),
                0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                currentVelocityDriftPerTimeUnit1.getUnit());
        final Acceleration currentVelocityDriftPerTimeUnit2 =
                new Acceleration(1.0,
                        AccelerationUnit.FEET_PER_SQUARED_SECOND);
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                currentVelocityDriftPerTimeUnit2));
        assertEquals(currentVelocityDriftPerTimeUnit1,
                currentVelocityDriftPerTimeUnit2);

        final Double currentOrientationDriftPerTimeUnit =
                kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit();
        assertNotNull(currentOrientationDriftPerTimeUnit);
        LOGGER.log(Level.INFO, "Current orientation drift per time unit: " +
                angularSpeedFormatter.format(AngularSpeedConverter.convert(
                        currentOrientationDriftPerTimeUnit,
                        AngularSpeedUnit.RADIANS_PER_SECOND,
                        AngularSpeedUnit.DEGREES_PER_SECOND),
                        AngularSpeedUnit.DEGREES_PER_SECOND));
        assertTrue(currentOrientationDriftPerTimeUnit < 2e-4);
        assertEquals(currentOrientationDriftPerTimeUnit,
                currentOrientationDriftNorm / (N_SAMPLES * TIME_INTERVAL_SECONDS),
                ABSOLUTE_ERROR);

        final AngularSpeed currentOrientationDriftPerTimeUnit1 =
                kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed();
        assertEquals(currentOrientationDriftPerTimeUnit,
                currentOrientationDriftPerTimeUnit1.getValue().doubleValue(),
                0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND,
                currentOrientationDriftPerTimeUnit1.getUnit());
        final AngularSpeed currentOrientationDriftPerTimeUnit2 =
                new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                currentOrientationDriftPerTimeUnit2));
        assertEquals(currentOrientationDriftPerTimeUnit1,
                currentOrientationDriftPerTimeUnit2);

        // reset
        kalmanDriftEstimator.reset();

        assertEquals(mReset, 1);
        assertEquals(0, kalmanDriftEstimator.getNumberOfProcessedSamples());
        assertNull(kalmanDriftEstimator.getCurrentPositionDrift());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDrift());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDrift());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftNormMeters());
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftNorm());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDriftNorm(null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNorm());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftNorm(null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftRadians());
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftAngle());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftAngle(null));
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(
                null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
    }

    @Test
    public void testAddBodyKinematicsAndResetExactCalibrationNoNoiseAndKinematicsNotFixed()
            throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            InertialNavigatorException, DriftEstimationException,
            NotReadyException, InvalidRotationMatrixException,
            RotationException, INSException {

        final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig = generateInitConfig();

        final Matrix ba = generateBa();
        final Matrix ma = generateMaCommonAxis();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = 0.0;
        final double gyroNoiseRootPSD = 0.0;
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        final NEDFrame nedFrame = generateFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final Random random = new Random();

        final CoordinateTransformation ecefC = ecefFrame
                .getCoordinateTransformation();
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final ECEFFrame navigationFrame = new ECEFFrame(ecefFrame);

        final KalmanDriftEstimator kalmanDriftEstimator = new KalmanDriftEstimator(
                nedFrame, ba, ma, bg, mg, gg, kalmanConfig, initConfig, this);
        kalmanDriftEstimator.setFixKinematicsEnabled(false);
        kalmanDriftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

        final DriftEstimator driftEstimator = new DriftEstimator(nedFrame,
                ba, ma, bg, mg, gg);
        driftEstimator.setFixKinematicsEnabled(false);
        driftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

        final INSLooselyCoupledKalmanFilteredEstimator kalmanEstimator =
                new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                        initConfig, new ECEFFrame(ecefFrame));

        reset();
        assertEquals(mStart, 0);
        assertEquals(mBodyKinematicsAdded, 0);
        assertEquals(mReset, 0);
        assertEquals(kalmanDriftEstimator.getNumberOfProcessedSamples(), 0);
        assertFalse(kalmanDriftEstimator.isRunning());
        assertFalse(kalmanDriftEstimator.isFixKinematicsEnabled());

        final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                        ecefC, ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition);

        final BodyKinematics measuredKinematics = new BodyKinematics();
        for (int i = 0; i < N_SAMPLES; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);

            kalmanDriftEstimator.addBodyKinematics(measuredKinematics);
            driftEstimator.addBodyKinematics(measuredKinematics);

            assertEquals(kalmanDriftEstimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(kalmanDriftEstimator.isRunning());

            ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                    navigationFrame, measuredKinematics, navigationFrame);

            kalmanEstimator.update(measuredKinematics, i * TIME_INTERVAL_SECONDS);
        }

        assertEquals(kalmanDriftEstimator.getNumberOfProcessedSamples(), N_SAMPLES);
        assertFalse(kalmanDriftEstimator.isRunning());
        assertEquals(mStart, 1);
        assertEquals(mBodyKinematicsAdded, N_SAMPLES);
        assertEquals(mReset, 0);

        final INSLooselyCoupledKalmanState kalmanState = kalmanEstimator.getState();

        final double navigationPositionDrift = positionDrift(
                ecefFrame, navigationFrame);
        final double navigationVelocityDrift = velocityDrift(
                ecefFrame, navigationFrame);
        final double navigationOrientationDrift = orientationDrift(
                ecefFrame, navigationFrame);

        final double kalmanPositionDrift = positionDrift(
                ecefFrame, kalmanState);
        final double kalmanVelocityDrift = velocityDrift(
                ecefFrame, kalmanState);
        final double kalmanOrientationDrift = orientationDrift(
                ecefFrame, kalmanState);

        final ECEFPosition currentPositionDrift1 = kalmanDriftEstimator
                .getCurrentPositionDrift();
        final ECEFPosition currentPositionDrift2 = new ECEFPosition();
        assertTrue(kalmanDriftEstimator.getCurrentPositionDrift(currentPositionDrift2));
        assertEquals(currentPositionDrift1, currentPositionDrift2);

        final ECEFVelocity currentVelocityDrift1 = kalmanDriftEstimator
                .getCurrentVelocityDrift();
        final ECEFVelocity currentVelocityDrift2 = new ECEFVelocity();
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDrift(currentVelocityDrift2));
        assertEquals(currentVelocityDrift1, currentVelocityDrift2);

        final Rotation3D currentOrientationDrift1 = kalmanDriftEstimator
                .getCurrentOrientationDrift();
        final Quaternion currentOrientationDrift2 = new Quaternion();
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDrift(currentOrientationDrift2));
        assertEquals(currentOrientationDrift1, currentOrientationDrift2);

        final DistanceFormatter distanceFormatter = new DistanceFormatter();
        final SpeedFormatter speedFormatter = new SpeedFormatter();
        final AngleFormatter angleFormatter = new AngleFormatter();
        final AccelerationFormatter accelerationFormatter =
                new AccelerationFormatter();
        final AngularSpeedFormatter angularSpeedFormatter =
                new AngularSpeedFormatter();

        final Double currentPositionDriftNorm = kalmanDriftEstimator
                .getCurrentPositionDriftNormMeters();
        assertNotNull(currentPositionDriftNorm);
        assertEquals(currentPositionDriftNorm, currentPositionDrift1.getNorm(),
                0.0);
        LOGGER.log(Level.INFO, "Current position drift: " +
                distanceFormatter.format(currentPositionDriftNorm,
                        DistanceUnit.METER));
        assertTrue(currentPositionDriftNorm < 17.0);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentPositionDriftNorm < navigationPositionDrift);
        assertEquals(currentPositionDriftNorm, kalmanPositionDrift,
                ABSOLUTE_ERROR);

        final Distance currentPositionDriftNorm1 = kalmanDriftEstimator
                .getCurrentPositionDriftNorm();
        assertEquals(currentPositionDriftNorm,
                currentPositionDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, currentPositionDriftNorm1.getUnit());
        final Distance currentPositionDriftNorm2 = new Distance(1.0,
                DistanceUnit.FOOT);
        assertTrue(kalmanDriftEstimator.getCurrentPositionDriftNorm(
                currentPositionDriftNorm2));
        assertEquals(currentPositionDriftNorm1, currentPositionDriftNorm2);

        final Double currentVelocityDriftNorm = kalmanDriftEstimator
                .getCurrentVelocityDriftNormMetersPerSecond();
        assertNotNull(currentVelocityDriftNorm);
        assertEquals(currentVelocityDriftNorm, currentVelocityDrift1.getNorm(),
                0.0);
        LOGGER.log(Level.INFO, "Current velocity drift: " +
                speedFormatter.format(currentVelocityDriftNorm,
                        SpeedUnit.METERS_PER_SECOND));
        assertTrue(currentVelocityDriftNorm < 1.3);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentVelocityDriftNorm < navigationVelocityDrift);
        assertEquals(currentVelocityDriftNorm, kalmanVelocityDrift,
                ABSOLUTE_ERROR);

        final Speed currentVelocityDriftNorm1 = kalmanDriftEstimator
                .getCurrentVelocityDriftNorm();
        assertEquals(currentVelocityDriftNorm,
                currentVelocityDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND,
                currentVelocityDriftNorm1.getUnit());
        final Speed currentVelocityDriftNorm2 = new Speed(1.0,
                SpeedUnit.KILOMETERS_PER_HOUR);
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftNorm(
                currentVelocityDriftNorm2));
        assertEquals(currentVelocityDriftNorm1, currentVelocityDriftNorm2);

        final Double currentOrientationDriftNorm = kalmanDriftEstimator
                .getCurrentOrientationDriftRadians();
        assertNotNull(currentOrientationDriftNorm);
        assertEquals(currentOrientationDriftNorm,
                currentOrientationDrift1.getRotationAngle(), 0.0);
        LOGGER.log(Level.INFO, "Current orientation drift: " +
                angleFormatter.format(AngleConverter.convert(
                        currentOrientationDriftNorm, AngleUnit.RADIANS,
                        AngleUnit.DEGREES), AngleUnit.DEGREES));
        assertTrue(currentOrientationDriftNorm < 0.008);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentOrientationDriftNorm < navigationOrientationDrift);
        assertEquals(currentOrientationDriftNorm, kalmanOrientationDrift,
                ABSOLUTE_ERROR);

        final Angle currentOrientationDriftNorm1 = kalmanDriftEstimator
                .getCurrentOrientationDriftAngle();
        assertEquals(currentOrientationDriftNorm,
                currentOrientationDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, currentOrientationDriftNorm1.getUnit());
        final Angle currentOrientationDriftNorm2 = new Angle(1.0,
                AngleUnit.DEGREES);
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftAngle(
                currentOrientationDriftNorm2));
        assertEquals(currentOrientationDriftNorm1, currentOrientationDriftNorm2);

        final Double currentPositionDriftPerTimeUnit =
                kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit();
        assertNotNull(currentPositionDriftPerTimeUnit);
        LOGGER.log(Level.INFO, "Current position drift per time unit: " +
                speedFormatter.format(currentPositionDriftPerTimeUnit,
                        SpeedUnit.METERS_PER_SECOND));
        assertEquals(currentPositionDriftPerTimeUnit,
                currentPositionDriftNorm / (N_SAMPLES * TIME_INTERVAL_SECONDS),
                ABSOLUTE_ERROR);

        final Speed currentPositionDriftPerTimeUnit1 =
                kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed();
        assertEquals(currentPositionDriftPerTimeUnit,
                currentPositionDriftPerTimeUnit1.getValue().doubleValue(),
                0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND,
                currentPositionDriftPerTimeUnit1.getUnit());
        final Speed currentPositionDriftPerTimeUnit2 = new Speed(1.0,
                SpeedUnit.KILOMETERS_PER_HOUR);
        assertTrue(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(
                currentPositionDriftPerTimeUnit2));
        assertEquals(currentPositionDriftPerTimeUnit1,
                currentPositionDriftPerTimeUnit2);

        final Double currentVelocityDriftPerTimeUnit =
                kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit();
        assertNotNull(currentVelocityDriftPerTimeUnit);
        LOGGER.log(Level.INFO, "Current velocity drift per time unit: " +
                accelerationFormatter.format(currentVelocityDriftPerTimeUnit,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(currentVelocityDriftPerTimeUnit,
                currentVelocityDriftNorm / (N_SAMPLES * TIME_INTERVAL_SECONDS),
                ABSOLUTE_ERROR);

        final Acceleration currentVelocityDriftPerTimeUnit1 =
                kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration();
        assertEquals(currentVelocityDriftPerTimeUnit,
                currentVelocityDriftPerTimeUnit1.getValue().doubleValue(),
                0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                currentVelocityDriftPerTimeUnit1.getUnit());
        final Acceleration currentVelocityDriftPerTimeUnit2 =
                new Acceleration(1.0,
                        AccelerationUnit.FEET_PER_SQUARED_SECOND);
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                currentVelocityDriftPerTimeUnit2));
        assertEquals(currentVelocityDriftPerTimeUnit1,
                currentVelocityDriftPerTimeUnit2);

        final Double currentOrientationDriftPerTimeUnit =
                kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit();
        assertNotNull(currentOrientationDriftPerTimeUnit);
        LOGGER.log(Level.INFO, "Current orientation drift per time unit: " +
                angularSpeedFormatter.format(AngularSpeedConverter.convert(
                        currentOrientationDriftPerTimeUnit,
                        AngularSpeedUnit.RADIANS_PER_SECOND,
                        AngularSpeedUnit.DEGREES_PER_SECOND),
                        AngularSpeedUnit.DEGREES_PER_SECOND));
        assertEquals(currentOrientationDriftPerTimeUnit,
                currentOrientationDriftNorm / (N_SAMPLES * TIME_INTERVAL_SECONDS),
                ABSOLUTE_ERROR);

        final AngularSpeed currentOrientationDriftPerTimeUnit1 =
                kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed();
        assertEquals(currentOrientationDriftPerTimeUnit,
                currentOrientationDriftPerTimeUnit1.getValue().doubleValue(),
                0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND,
                currentOrientationDriftPerTimeUnit1.getUnit());
        final AngularSpeed currentOrientationDriftPerTimeUnit2 =
                new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                currentOrientationDriftPerTimeUnit2));
        assertEquals(currentOrientationDriftPerTimeUnit1,
                currentOrientationDriftPerTimeUnit2);

        // reset
        kalmanDriftEstimator.reset();

        assertEquals(mReset, 1);
        assertEquals(0, kalmanDriftEstimator.getNumberOfProcessedSamples());
        assertNull(kalmanDriftEstimator.getCurrentPositionDrift());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDrift());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDrift());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftNormMeters());
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftNorm());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDriftNorm(null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNorm());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftNorm(null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftRadians());
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftAngle());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftAngle(null));
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(
                null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
    }

    @Test
    public void testAddBodyKinematicsAndResetExactCalibrationWithNoiseAndKinematicsFixed()
            throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            InertialNavigatorException, DriftEstimationException,
            NotReadyException, InvalidRotationMatrixException,
            RotationException, INSException {

        final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig = generateInitConfig();

        final Matrix ba = generateBa();
        final Matrix ma = generateMaCommonAxis();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        final NEDFrame nedFrame = generateFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final Random random = new Random();

        final CoordinateTransformation ecefC = ecefFrame
                .getCoordinateTransformation();
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final ECEFFrame navigationFrame = new ECEFFrame(ecefFrame);
        final BodyKinematicsFixer fixer = new BodyKinematicsFixer();
        fixer.setAccelerationBias(ba);
        fixer.setAccelerationCrossCouplingErrors(ma);
        fixer.setAngularSpeedBias(bg);
        fixer.setAngularSpeedCrossCouplingErrors(mg);
        fixer.setAngularSpeedGDependantCrossBias(gg);

        final KalmanDriftEstimator kalmanDriftEstimator = new KalmanDriftEstimator(
                nedFrame, ba, ma, bg, mg, gg, kalmanConfig, initConfig, this);
        kalmanDriftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

        final DriftEstimator driftEstimator = new DriftEstimator(nedFrame,
                ba, ma, bg, mg, gg);
        driftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

        final INSLooselyCoupledKalmanFilteredEstimator kalmanEstimator =
                new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                        initConfig, new ECEFFrame(ecefFrame));

        reset();
        assertEquals(mStart, 0);
        assertEquals(mBodyKinematicsAdded, 0);
        assertEquals(mReset, 0);
        assertEquals(kalmanDriftEstimator.getNumberOfProcessedSamples(), 0);
        assertFalse(kalmanDriftEstimator.isRunning());
        assertTrue(kalmanDriftEstimator.isFixKinematicsEnabled());

        final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                        ecefC, ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition);
        final BodyKinematics fixedKinematics = new BodyKinematics();

        final BodyKinematics measuredKinematics = new BodyKinematics();
        for (int i = 0; i < N_SAMPLES; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);

            kalmanDriftEstimator.addBodyKinematics(measuredKinematics);
            driftEstimator.addBodyKinematics(measuredKinematics);

            assertEquals(kalmanDriftEstimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(kalmanDriftEstimator.isRunning());

            fixer.fix(measuredKinematics, fixedKinematics);
            ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                    navigationFrame, fixedKinematics, navigationFrame);

            kalmanEstimator.update(fixedKinematics, i * TIME_INTERVAL_SECONDS);
        }

        assertEquals(kalmanDriftEstimator.getNumberOfProcessedSamples(), N_SAMPLES);
        assertFalse(kalmanDriftEstimator.isRunning());
        assertEquals(mStart, 1);
        assertEquals(mBodyKinematicsAdded, N_SAMPLES);
        assertEquals(mReset, 0);

        final INSLooselyCoupledKalmanState kalmanState = kalmanEstimator.getState();

        final double navigationPositionDrift = positionDrift(
                ecefFrame, navigationFrame);
        final double navigationVelocityDrift = velocityDrift(
                ecefFrame, navigationFrame);
        final double navigationOrientationDrift = orientationDrift(
                ecefFrame, navigationFrame);

        final double kalmanPositionDrift = positionDrift(
                ecefFrame, kalmanState);
        final double kalmanVelocityDrift = velocityDrift(
                ecefFrame, kalmanState);
        final double kalmanOrientationDrift = orientationDrift(
                ecefFrame, kalmanState);

        final ECEFPosition currentPositionDrift1 = kalmanDriftEstimator
                .getCurrentPositionDrift();
        final ECEFPosition currentPositionDrift2 = new ECEFPosition();
        assertTrue(kalmanDriftEstimator.getCurrentPositionDrift(currentPositionDrift2));
        assertEquals(currentPositionDrift1, currentPositionDrift2);

        final ECEFVelocity currentVelocityDrift1 = kalmanDriftEstimator
                .getCurrentVelocityDrift();
        final ECEFVelocity currentVelocityDrift2 = new ECEFVelocity();
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDrift(currentVelocityDrift2));
        assertEquals(currentVelocityDrift1, currentVelocityDrift2);

        final Rotation3D currentOrientationDrift1 = kalmanDriftEstimator
                .getCurrentOrientationDrift();
        final Quaternion currentOrientationDrift2 = new Quaternion();
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDrift(currentOrientationDrift2));
        assertEquals(currentOrientationDrift1, currentOrientationDrift2);

        final DistanceFormatter distanceFormatter = new DistanceFormatter();
        final SpeedFormatter speedFormatter = new SpeedFormatter();
        final AngleFormatter angleFormatter = new AngleFormatter();
        final AccelerationFormatter accelerationFormatter =
                new AccelerationFormatter();
        final AngularSpeedFormatter angularSpeedFormatter =
                new AngularSpeedFormatter();

        final Double currentPositionDriftNorm = kalmanDriftEstimator
                .getCurrentPositionDriftNormMeters();
        assertNotNull(currentPositionDriftNorm);
        assertEquals(currentPositionDriftNorm, currentPositionDrift1.getNorm(),
                0.0);
        LOGGER.log(Level.INFO, "Current position drift: " +
                distanceFormatter.format(currentPositionDriftNorm,
                        DistanceUnit.METER));
        assertTrue(currentPositionDriftNorm < 8.0);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentPositionDriftNorm < navigationPositionDrift);
        assertEquals(currentPositionDriftNorm, kalmanPositionDrift,
                ABSOLUTE_ERROR);

        final Distance currentPositionDriftNorm1 = kalmanDriftEstimator
                .getCurrentPositionDriftNorm();
        assertEquals(currentPositionDriftNorm,
                currentPositionDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, currentPositionDriftNorm1.getUnit());
        final Distance currentPositionDriftNorm2 = new Distance(1.0,
                DistanceUnit.FOOT);
        assertTrue(kalmanDriftEstimator.getCurrentPositionDriftNorm(
                currentPositionDriftNorm2));
        assertEquals(currentPositionDriftNorm1, currentPositionDriftNorm2);

        final Double currentVelocityDriftNorm = kalmanDriftEstimator
                .getCurrentVelocityDriftNormMetersPerSecond();
        assertNotNull(currentVelocityDriftNorm);
        assertEquals(currentVelocityDriftNorm, currentVelocityDrift1.getNorm(),
                0.0);
        LOGGER.log(Level.INFO, "Current velocity drift: " +
                speedFormatter.format(currentVelocityDriftNorm,
                        SpeedUnit.METERS_PER_SECOND));
        assertTrue(currentVelocityDriftNorm < 0.7);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentVelocityDriftNorm < navigationVelocityDrift);
        assertEquals(currentVelocityDriftNorm, kalmanVelocityDrift,
                ABSOLUTE_ERROR);

        final Speed currentVelocityDriftNorm1 = kalmanDriftEstimator
                .getCurrentVelocityDriftNorm();
        assertEquals(currentVelocityDriftNorm,
                currentVelocityDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND,
                currentVelocityDriftNorm1.getUnit());
        final Speed currentVelocityDriftNorm2 = new Speed(1.0,
                SpeedUnit.KILOMETERS_PER_HOUR);
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftNorm(
                currentVelocityDriftNorm2));
        assertEquals(currentVelocityDriftNorm1, currentVelocityDriftNorm2);

        final Double currentOrientationDriftNorm = kalmanDriftEstimator
                .getCurrentOrientationDriftRadians();
        assertNotNull(currentOrientationDriftNorm);
        assertEquals(currentOrientationDriftNorm,
                currentOrientationDrift1.getRotationAngle(), 0.0);
        LOGGER.log(Level.INFO, "Current orientation drift: " +
                angleFormatter.format(AngleConverter.convert(
                        currentOrientationDriftNorm, AngleUnit.RADIANS,
                        AngleUnit.DEGREES), AngleUnit.DEGREES));
        assertTrue(currentOrientationDriftNorm < 0.006);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentOrientationDriftNorm < navigationOrientationDrift);
        assertEquals(currentOrientationDriftNorm, kalmanOrientationDrift,
                ABSOLUTE_ERROR);

        final Angle currentOrientationDriftNorm1 = kalmanDriftEstimator
                .getCurrentOrientationDriftAngle();
        assertEquals(currentOrientationDriftNorm,
                currentOrientationDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, currentOrientationDriftNorm1.getUnit());
        final Angle currentOrientationDriftNorm2 = new Angle(1.0,
                AngleUnit.DEGREES);
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftAngle(
                currentOrientationDriftNorm2));
        assertEquals(currentOrientationDriftNorm1, currentOrientationDriftNorm2);

        final Double currentPositionDriftPerTimeUnit =
                kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit();
        assertNotNull(currentPositionDriftPerTimeUnit);
        LOGGER.log(Level.INFO, "Current position drift per time unit: " +
                speedFormatter.format(currentPositionDriftPerTimeUnit,
                        SpeedUnit.METERS_PER_SECOND));
        assertTrue(currentPositionDriftPerTimeUnit < 0.3);
        assertEquals(currentPositionDriftPerTimeUnit,
                currentPositionDriftNorm / (N_SAMPLES * TIME_INTERVAL_SECONDS),
                ABSOLUTE_ERROR);

        final Speed currentPositionDriftPerTimeUnit1 =
                kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed();
        assertEquals(currentPositionDriftPerTimeUnit,
                currentPositionDriftPerTimeUnit1.getValue().doubleValue(),
                0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND,
                currentPositionDriftPerTimeUnit1.getUnit());
        final Speed currentPositionDriftPerTimeUnit2 = new Speed(1.0,
                SpeedUnit.KILOMETERS_PER_HOUR);
        assertTrue(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(
                currentPositionDriftPerTimeUnit2));
        assertEquals(currentPositionDriftPerTimeUnit1,
                currentPositionDriftPerTimeUnit2);

        final Double currentVelocityDriftPerTimeUnit =
                kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit();
        assertNotNull(currentVelocityDriftPerTimeUnit);
        LOGGER.log(Level.INFO, "Current velocity drift per time unit: " +
                accelerationFormatter.format(currentVelocityDriftPerTimeUnit,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertTrue(currentVelocityDriftPerTimeUnit < 0.03);
        assertEquals(currentVelocityDriftPerTimeUnit,
                currentVelocityDriftNorm / (N_SAMPLES * TIME_INTERVAL_SECONDS),
                ABSOLUTE_ERROR);

        final Acceleration currentVelocityDriftPerTimeUnit1 =
                kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration();
        assertEquals(currentVelocityDriftPerTimeUnit,
                currentVelocityDriftPerTimeUnit1.getValue().doubleValue(),
                0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                currentVelocityDriftPerTimeUnit1.getUnit());
        final Acceleration currentVelocityDriftPerTimeUnit2 =
                new Acceleration(1.0,
                        AccelerationUnit.FEET_PER_SQUARED_SECOND);
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                currentVelocityDriftPerTimeUnit2));
        assertEquals(currentVelocityDriftPerTimeUnit1,
                currentVelocityDriftPerTimeUnit2);

        final Double currentOrientationDriftPerTimeUnit =
                kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit();
        assertNotNull(currentOrientationDriftPerTimeUnit);
        LOGGER.log(Level.INFO, "Current orientation drift per time unit: " +
                angularSpeedFormatter.format(AngularSpeedConverter.convert(
                        currentOrientationDriftPerTimeUnit,
                        AngularSpeedUnit.RADIANS_PER_SECOND,
                        AngularSpeedUnit.DEGREES_PER_SECOND),
                        AngularSpeedUnit.DEGREES_PER_SECOND));
        assertTrue(currentOrientationDriftPerTimeUnit < 2e-4);
        assertEquals(currentOrientationDriftPerTimeUnit,
                currentOrientationDriftNorm / (N_SAMPLES * TIME_INTERVAL_SECONDS),
                ABSOLUTE_ERROR);

        final AngularSpeed currentOrientationDriftPerTimeUnit1 =
                kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed();
        assertEquals(currentOrientationDriftPerTimeUnit,
                currentOrientationDriftPerTimeUnit1.getValue().doubleValue(),
                0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND,
                currentOrientationDriftPerTimeUnit1.getUnit());
        final AngularSpeed currentOrientationDriftPerTimeUnit2 =
                new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                currentOrientationDriftPerTimeUnit2));
        assertEquals(currentOrientationDriftPerTimeUnit1,
                currentOrientationDriftPerTimeUnit2);

        // reset
        kalmanDriftEstimator.reset();

        assertEquals(mReset, 1);
        assertEquals(0, kalmanDriftEstimator.getNumberOfProcessedSamples());
        assertNull(kalmanDriftEstimator.getCurrentPositionDrift());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDrift());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDrift());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftNormMeters());
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftNorm());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDriftNorm(null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNorm());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftNorm(null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftRadians());
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftAngle());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftAngle(null));
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(
                null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
    }

    @Test
    public void testAddBodyKinematicsAndResetExactCalibrationWithNoiseAndKinematicsNotFixed()
            throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            InertialNavigatorException, DriftEstimationException,
            NotReadyException, InvalidRotationMatrixException,
            RotationException, INSException {

        final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
        final INSLooselyCoupledKalmanInitializerConfig initConfig = generateInitConfig();

        final Matrix ba = generateBa();
        final Matrix ma = generateMaCommonAxis();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        final NEDFrame nedFrame = generateFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final Random random = new Random();

        final CoordinateTransformation ecefC = ecefFrame
                .getCoordinateTransformation();
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final ECEFFrame navigationFrame = new ECEFFrame(ecefFrame);

        final KalmanDriftEstimator kalmanDriftEstimator = new KalmanDriftEstimator(
                nedFrame, ba, ma, bg, mg, gg, kalmanConfig, initConfig, this);
        kalmanDriftEstimator.setFixKinematicsEnabled(false);
        kalmanDriftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

        final DriftEstimator driftEstimator = new DriftEstimator(nedFrame,
                ba, ma, bg, mg, gg);
        driftEstimator.setFixKinematicsEnabled(false);
        driftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

        final INSLooselyCoupledKalmanFilteredEstimator kalmanEstimator =
                new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                        initConfig, new ECEFFrame(ecefFrame));

        reset();
        assertEquals(mStart, 0);
        assertEquals(mBodyKinematicsAdded, 0);
        assertEquals(mReset, 0);
        assertEquals(kalmanDriftEstimator.getNumberOfProcessedSamples(), 0);
        assertFalse(kalmanDriftEstimator.isRunning());
        assertFalse(kalmanDriftEstimator.isFixKinematicsEnabled());

        final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                        ecefC, ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition);

        final BodyKinematics measuredKinematics = new BodyKinematics();
        for (int i = 0; i < N_SAMPLES; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);

            kalmanDriftEstimator.addBodyKinematics(measuredKinematics);
            driftEstimator.addBodyKinematics(measuredKinematics);

            assertEquals(kalmanDriftEstimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(kalmanDriftEstimator.isRunning());

            ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                    navigationFrame, measuredKinematics, navigationFrame);

            kalmanEstimator.update(measuredKinematics, i * TIME_INTERVAL_SECONDS);
        }

        assertEquals(kalmanDriftEstimator.getNumberOfProcessedSamples(), N_SAMPLES);
        assertFalse(kalmanDriftEstimator.isRunning());
        assertEquals(mStart, 1);
        assertEquals(mBodyKinematicsAdded, N_SAMPLES);
        assertEquals(mReset, 0);

        final INSLooselyCoupledKalmanState kalmanState = kalmanEstimator.getState();

        final double navigationPositionDrift = positionDrift(
                ecefFrame, navigationFrame);
        final double navigationVelocityDrift = velocityDrift(
                ecefFrame, navigationFrame);
        final double navigationOrientationDrift = orientationDrift(
                ecefFrame, navigationFrame);

        final double kalmanPositionDrift = positionDrift(
                ecefFrame, kalmanState);
        final double kalmanVelocityDrift = velocityDrift(
                ecefFrame, kalmanState);
        final double kalmanOrientationDrift = orientationDrift(
                ecefFrame, kalmanState);

        final ECEFPosition currentPositionDrift1 = kalmanDriftEstimator
                .getCurrentPositionDrift();
        final ECEFPosition currentPositionDrift2 = new ECEFPosition();
        assertTrue(kalmanDriftEstimator.getCurrentPositionDrift(currentPositionDrift2));
        assertEquals(currentPositionDrift1, currentPositionDrift2);

        final ECEFVelocity currentVelocityDrift1 = kalmanDriftEstimator
                .getCurrentVelocityDrift();
        final ECEFVelocity currentVelocityDrift2 = new ECEFVelocity();
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDrift(currentVelocityDrift2));
        assertEquals(currentVelocityDrift1, currentVelocityDrift2);

        final Rotation3D currentOrientationDrift1 = kalmanDriftEstimator
                .getCurrentOrientationDrift();
        final Quaternion currentOrientationDrift2 = new Quaternion();
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDrift(currentOrientationDrift2));
        assertEquals(currentOrientationDrift1, currentOrientationDrift2);

        final DistanceFormatter distanceFormatter = new DistanceFormatter();
        final SpeedFormatter speedFormatter = new SpeedFormatter();
        final AngleFormatter angleFormatter = new AngleFormatter();
        final AccelerationFormatter accelerationFormatter =
                new AccelerationFormatter();
        final AngularSpeedFormatter angularSpeedFormatter =
                new AngularSpeedFormatter();

        final Double currentPositionDriftNorm = kalmanDriftEstimator
                .getCurrentPositionDriftNormMeters();
        assertNotNull(currentPositionDriftNorm);
        assertEquals(currentPositionDriftNorm, currentPositionDrift1.getNorm(),
                0.0);
        LOGGER.log(Level.INFO, "Current position drift: " +
                distanceFormatter.format(currentPositionDriftNorm,
                        DistanceUnit.METER));
        assertTrue(currentPositionDriftNorm < 19.0);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentPositionDriftNorm < navigationPositionDrift);
        assertEquals(currentPositionDriftNorm, kalmanPositionDrift,
                ABSOLUTE_ERROR);

        final Distance currentPositionDriftNorm1 = kalmanDriftEstimator
                .getCurrentPositionDriftNorm();
        assertEquals(currentPositionDriftNorm,
                currentPositionDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, currentPositionDriftNorm1.getUnit());
        final Distance currentPositionDriftNorm2 = new Distance(1.0,
                DistanceUnit.FOOT);
        assertTrue(kalmanDriftEstimator.getCurrentPositionDriftNorm(
                currentPositionDriftNorm2));
        assertEquals(currentPositionDriftNorm1, currentPositionDriftNorm2);

        final Double currentVelocityDriftNorm = kalmanDriftEstimator
                .getCurrentVelocityDriftNormMetersPerSecond();
        assertNotNull(currentVelocityDriftNorm);
        assertEquals(currentVelocityDriftNorm, currentVelocityDrift1.getNorm(),
                0.0);
        LOGGER.log(Level.INFO, "Current velocity drift: " +
                speedFormatter.format(currentVelocityDriftNorm,
                        SpeedUnit.METERS_PER_SECOND));
        assertTrue(currentVelocityDriftNorm < 1.4);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentVelocityDriftNorm < navigationVelocityDrift);
        assertEquals(currentVelocityDriftNorm, kalmanVelocityDrift,
                ABSOLUTE_ERROR);

        final Speed currentVelocityDriftNorm1 = kalmanDriftEstimator
                .getCurrentVelocityDriftNorm();
        assertEquals(currentVelocityDriftNorm,
                currentVelocityDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND,
                currentVelocityDriftNorm1.getUnit());
        final Speed currentVelocityDriftNorm2 = new Speed(1.0,
                SpeedUnit.KILOMETERS_PER_HOUR);
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftNorm(
                currentVelocityDriftNorm2));
        assertEquals(currentVelocityDriftNorm1, currentVelocityDriftNorm2);

        final Double currentOrientationDriftNorm = kalmanDriftEstimator
                .getCurrentOrientationDriftRadians();
        assertNotNull(currentOrientationDriftNorm);
        assertEquals(currentOrientationDriftNorm,
                currentOrientationDrift1.getRotationAngle(), 0.0);
        LOGGER.log(Level.INFO, "Current orientation drift: " +
                angleFormatter.format(AngleConverter.convert(
                        currentOrientationDriftNorm, AngleUnit.RADIANS,
                        AngleUnit.DEGREES), AngleUnit.DEGREES));
        assertTrue(currentOrientationDriftNorm < 0.007);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentOrientationDriftNorm < navigationOrientationDrift);
        assertEquals(currentOrientationDriftNorm, kalmanOrientationDrift,
                ABSOLUTE_ERROR);

        final Angle currentOrientationDriftNorm1 = kalmanDriftEstimator
                .getCurrentOrientationDriftAngle();
        assertEquals(currentOrientationDriftNorm,
                currentOrientationDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, currentOrientationDriftNorm1.getUnit());
        final Angle currentOrientationDriftNorm2 = new Angle(1.0,
                AngleUnit.DEGREES);
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftAngle(
                currentOrientationDriftNorm2));
        assertEquals(currentOrientationDriftNorm1, currentOrientationDriftNorm2);

        final Double currentPositionDriftPerTimeUnit =
                kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit();
        assertNotNull(currentPositionDriftPerTimeUnit);
        LOGGER.log(Level.INFO, "Current position drift per time unit: " +
                speedFormatter.format(currentPositionDriftPerTimeUnit,
                        SpeedUnit.METERS_PER_SECOND));
        assertEquals(currentPositionDriftPerTimeUnit,
                currentPositionDriftNorm / (N_SAMPLES * TIME_INTERVAL_SECONDS),
                ABSOLUTE_ERROR);

        final Speed currentPositionDriftPerTimeUnit1 =
                kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed();
        assertEquals(currentPositionDriftPerTimeUnit,
                currentPositionDriftPerTimeUnit1.getValue().doubleValue(),
                0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND,
                currentPositionDriftPerTimeUnit1.getUnit());
        final Speed currentPositionDriftPerTimeUnit2 = new Speed(1.0,
                SpeedUnit.KILOMETERS_PER_HOUR);
        assertTrue(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(
                currentPositionDriftPerTimeUnit2));
        assertEquals(currentPositionDriftPerTimeUnit1,
                currentPositionDriftPerTimeUnit2);

        final Double currentVelocityDriftPerTimeUnit =
                kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit();
        assertNotNull(currentVelocityDriftPerTimeUnit);
        LOGGER.log(Level.INFO, "Current velocity drift per time unit: " +
                accelerationFormatter.format(currentVelocityDriftPerTimeUnit,
                        AccelerationUnit.METERS_PER_SQUARED_SECOND));
        assertEquals(currentVelocityDriftPerTimeUnit,
                currentVelocityDriftNorm / (N_SAMPLES * TIME_INTERVAL_SECONDS),
                ABSOLUTE_ERROR);

        final Acceleration currentVelocityDriftPerTimeUnit1 =
                kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration();
        assertEquals(currentVelocityDriftPerTimeUnit,
                currentVelocityDriftPerTimeUnit1.getValue().doubleValue(),
                0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                currentVelocityDriftPerTimeUnit1.getUnit());
        final Acceleration currentVelocityDriftPerTimeUnit2 =
                new Acceleration(1.0,
                        AccelerationUnit.FEET_PER_SQUARED_SECOND);
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                currentVelocityDriftPerTimeUnit2));
        assertEquals(currentVelocityDriftPerTimeUnit1,
                currentVelocityDriftPerTimeUnit2);

        final Double currentOrientationDriftPerTimeUnit =
                kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit();
        assertNotNull(currentOrientationDriftPerTimeUnit);
        LOGGER.log(Level.INFO, "Current orientation drift per time unit: " +
                angularSpeedFormatter.format(AngularSpeedConverter.convert(
                        currentOrientationDriftPerTimeUnit,
                        AngularSpeedUnit.RADIANS_PER_SECOND,
                        AngularSpeedUnit.DEGREES_PER_SECOND),
                        AngularSpeedUnit.DEGREES_PER_SECOND));
        assertEquals(currentOrientationDriftPerTimeUnit,
                currentOrientationDriftNorm / (N_SAMPLES * TIME_INTERVAL_SECONDS),
                ABSOLUTE_ERROR);

        final AngularSpeed currentOrientationDriftPerTimeUnit1 =
                kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed();
        assertEquals(currentOrientationDriftPerTimeUnit,
                currentOrientationDriftPerTimeUnit1.getValue().doubleValue(),
                0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND,
                currentOrientationDriftPerTimeUnit1.getUnit());
        final AngularSpeed currentOrientationDriftPerTimeUnit2 =
                new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                currentOrientationDriftPerTimeUnit2));
        assertEquals(currentOrientationDriftPerTimeUnit1,
                currentOrientationDriftPerTimeUnit2);

        // reset
        kalmanDriftEstimator.reset();

        assertEquals(mReset, 1);
        assertEquals(0, kalmanDriftEstimator.getNumberOfProcessedSamples());
        assertNull(kalmanDriftEstimator.getCurrentPositionDrift());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDrift());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDrift());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftNormMeters());
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftNorm());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDriftNorm(null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNorm());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftNorm(null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftRadians());
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftAngle());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftAngle(null));
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(
                null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                null));
    }

    @Test
    public void testAddBodyKinematicsAndResetApproximateCalibration()
            throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            InertialNavigatorException, DriftEstimationException,
            NotReadyException, InvalidRotationMatrixException,
            RotationException, INSException,
            IntervalDetectorThresholdFactorOptimizerException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final INSLooselyCoupledKalmanConfig kalmanConfig = generateKalmanConfig();
            final INSLooselyCoupledKalmanInitializerConfig initConfig = generateInitConfig();

            final Matrix ba = generateBa();
            final Matrix ma = generateMaCommonAxis();
            final Matrix bg = generateBg();
            final Matrix mg = generateMg();
            final Matrix gg =  new Matrix(3, 3);
            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator();
            final EasyGyroscopeCalibrator gyroscopeCalibrator =
                    new EasyGyroscopeCalibrator();

            // initialize a threshold optimizer to attempt calibration by generating
            // timed body kinematics with noise, and attempting to find the best
            // threshold to find optimal calibration
            final BracketedAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                    buildOptimizer(ba, ma, accelNoiseRootPSD, gyroNoiseRootPSD,
                            accelerometerCalibrator, gyroscopeCalibrator);
            final double thresholdFactor = optimizer.optimize();

            assertEquals(thresholdFactor, optimizer.getOptimalThresholdFactor(),
                    0.0);

            final Matrix estimatedBa = Matrix.newFromArray(
                    optimizer.getEstimatedAccelerometerBiases());
            final Matrix estimatedMa = optimizer.getEstimatedAccelerometerMa();

            final Matrix estimatedBg = Matrix.newFromArray(
                    optimizer.getEstimatedGyroscopeBiases());
            final Matrix estimatedMg = optimizer.getEstimatedGyroscopeMg();
            final Matrix estimatedGg = optimizer.getEstimatedGyroscopeGg();

            // clear all previously generated data
            mTimedBodyKinematics.clear();

            // use real calibration values to generate measurements with errors
            final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                    gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

            final NEDFrame nedFrame = generateFrame();
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            final Random random = new Random();

            final CoordinateTransformation ecefC = ecefFrame
                    .getCoordinateTransformation();
            final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

            final ECEFFrame navigationFrame = new ECEFFrame(ecefFrame);
            final BodyKinematicsFixer fixer = new BodyKinematicsFixer();
            // use estimated calibration data to fix measurements
            fixer.setAccelerationBias(estimatedBa);
            fixer.setAccelerationCrossCouplingErrors(estimatedMa);
            fixer.setAngularSpeedBias(estimatedBg);
            fixer.setAngularSpeedCrossCouplingErrors(estimatedMg);
            fixer.setAngularSpeedGDependantCrossBias(estimatedGg);

            final KalmanDriftEstimator kalmanDriftEstimator = new KalmanDriftEstimator(
                    nedFrame, estimatedBa, estimatedMa, estimatedBg,
                    estimatedMg, estimatedGg, kalmanConfig, initConfig,
                    this);
            kalmanDriftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

            final DriftEstimator driftEstimator = new DriftEstimator(nedFrame,
                    estimatedBa, estimatedMa, estimatedBg, estimatedMg,
                    estimatedGg);
            driftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

            final INSLooselyCoupledKalmanFilteredEstimator kalmanEstimator =
                    new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                            initConfig, new ECEFFrame(ecefFrame));

            reset();
            assertEquals(mStart, 0);
            assertEquals(mBodyKinematicsAdded, 0);
            assertEquals(mReset, 0);
            assertEquals(kalmanDriftEstimator.getNumberOfProcessedSamples(), 0);
            assertFalse(kalmanDriftEstimator.isRunning());
            assertTrue(kalmanDriftEstimator.isFixKinematicsEnabled());

            final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                            ecefC, ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition);
            final BodyKinematics fixedKinematics = new BodyKinematics();

            final BodyKinematics measuredKinematics = new BodyKinematics();
            for (int i = 0; i < N_SAMPLES; i++) {
                BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                        trueKinematics, errors, random, measuredKinematics);

                kalmanDriftEstimator.addBodyKinematics(measuredKinematics);
                driftEstimator.addBodyKinematics(measuredKinematics);

                assertEquals(kalmanDriftEstimator.getNumberOfProcessedSamples(), i + 1);
                assertFalse(kalmanDriftEstimator.isRunning());

                fixer.fix(measuredKinematics, fixedKinematics);
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                        navigationFrame, fixedKinematics, navigationFrame);

                kalmanEstimator.update(fixedKinematics, i * TIME_INTERVAL_SECONDS);
            }

            assertEquals(kalmanDriftEstimator.getNumberOfProcessedSamples(), N_SAMPLES);
            assertFalse(kalmanDriftEstimator.isRunning());
            assertEquals(mStart, 1);
            assertEquals(mBodyKinematicsAdded, N_SAMPLES);
            assertEquals(mReset, 0);

            final INSLooselyCoupledKalmanState kalmanState = kalmanEstimator.getState();

            final double navigationPositionDrift = positionDrift(
                    ecefFrame, navigationFrame);
            final double navigationVelocityDrift = velocityDrift(
                    ecefFrame, navigationFrame);
            final double navigationOrientationDrift = orientationDrift(
                    ecefFrame, navigationFrame);

            final double kalmanPositionDrift = positionDrift(
                    ecefFrame, kalmanState);
            final double kalmanVelocityDrift = velocityDrift(
                    ecefFrame, kalmanState);
            final double kalmanOrientationDrift = orientationDrift(
                    ecefFrame, kalmanState);

            final ECEFPosition currentPositionDrift1 = kalmanDriftEstimator
                    .getCurrentPositionDrift();
            final ECEFPosition currentPositionDrift2 = new ECEFPosition();
            assertTrue(kalmanDriftEstimator.getCurrentPositionDrift(currentPositionDrift2));
            assertEquals(currentPositionDrift1, currentPositionDrift2);

            final ECEFVelocity currentVelocityDrift1 = kalmanDriftEstimator
                    .getCurrentVelocityDrift();
            final ECEFVelocity currentVelocityDrift2 = new ECEFVelocity();
            assertTrue(kalmanDriftEstimator.getCurrentVelocityDrift(currentVelocityDrift2));
            assertEquals(currentVelocityDrift1, currentVelocityDrift2);

            final Rotation3D currentOrientationDrift1 = kalmanDriftEstimator
                    .getCurrentOrientationDrift();
            final Quaternion currentOrientationDrift2 = new Quaternion();
            assertTrue(kalmanDriftEstimator.getCurrentOrientationDrift(currentOrientationDrift2));
            assertEquals(currentOrientationDrift1, currentOrientationDrift2);

            final DistanceFormatter distanceFormatter = new DistanceFormatter();
            final SpeedFormatter speedFormatter = new SpeedFormatter();
            final AngleFormatter angleFormatter = new AngleFormatter();
            final AccelerationFormatter accelerationFormatter =
                    new AccelerationFormatter();
            final AngularSpeedFormatter angularSpeedFormatter =
                    new AngularSpeedFormatter();

            final Double currentPositionDriftNorm = kalmanDriftEstimator
                    .getCurrentPositionDriftNormMeters();
            assertNotNull(currentPositionDriftNorm);
            assertEquals(currentPositionDriftNorm, currentPositionDrift1.getNorm(),
                    0.0);
            LOGGER.log(Level.INFO, "Current position drift: " +
                    distanceFormatter.format(currentPositionDriftNorm,
                            DistanceUnit.METER));
            if (currentPositionDriftNorm >= 92.0) {
                continue;
            }
            assertTrue(currentPositionDriftNorm < 92.0);
            // Check that kalman corrected drift is smaller than normal navigation drift
            assertTrue(currentPositionDriftNorm < navigationPositionDrift);
            assertEquals(currentPositionDriftNorm, kalmanPositionDrift,
                    ABSOLUTE_ERROR);

            final Distance currentPositionDriftNorm1 = kalmanDriftEstimator
                    .getCurrentPositionDriftNorm();
            assertEquals(currentPositionDriftNorm,
                    currentPositionDriftNorm1.getValue().doubleValue(), 0.0);
            assertEquals(DistanceUnit.METER, currentPositionDriftNorm1.getUnit());
            final Distance currentPositionDriftNorm2 = new Distance(1.0,
                    DistanceUnit.FOOT);
            assertTrue(kalmanDriftEstimator.getCurrentPositionDriftNorm(
                    currentPositionDriftNorm2));
            assertEquals(currentPositionDriftNorm1, currentPositionDriftNorm2);

            final Double currentVelocityDriftNorm = kalmanDriftEstimator
                    .getCurrentVelocityDriftNormMetersPerSecond();
            assertNotNull(currentVelocityDriftNorm);
            assertEquals(currentVelocityDriftNorm, currentVelocityDrift1.getNorm(),
                    0.0);
            LOGGER.log(Level.INFO, "Current velocity drift: " +
                    speedFormatter.format(currentVelocityDriftNorm,
                            SpeedUnit.METERS_PER_SECOND));
            if (currentVelocityDriftNorm >= 6.0) {
                continue;
            }
            assertTrue(currentVelocityDriftNorm < 6.0);
            // Check that kalman corrected drift is smaller than normal navigation drift
            assertTrue(currentVelocityDriftNorm < navigationVelocityDrift);
            assertEquals(currentVelocityDriftNorm, kalmanVelocityDrift,
                    ABSOLUTE_ERROR);

            final Speed currentVelocityDriftNorm1 = kalmanDriftEstimator
                    .getCurrentVelocityDriftNorm();
            assertEquals(currentVelocityDriftNorm,
                    currentVelocityDriftNorm1.getValue().doubleValue(), 0.0);
            assertEquals(SpeedUnit.METERS_PER_SECOND,
                    currentVelocityDriftNorm1.getUnit());
            final Speed currentVelocityDriftNorm2 = new Speed(1.0,
                    SpeedUnit.KILOMETERS_PER_HOUR);
            assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftNorm(
                    currentVelocityDriftNorm2));
            assertEquals(currentVelocityDriftNorm1, currentVelocityDriftNorm2);

            final Double currentOrientationDriftNorm = kalmanDriftEstimator
                    .getCurrentOrientationDriftRadians();
            assertNotNull(currentOrientationDriftNorm);
            assertEquals(currentOrientationDriftNorm,
                    currentOrientationDrift1.getRotationAngle(), 0.0);
            LOGGER.log(Level.INFO, "Current orientation drift: " +
                    angleFormatter.format(AngleConverter.convert(
                            currentOrientationDriftNorm, AngleUnit.RADIANS,
                            AngleUnit.DEGREES), AngleUnit.DEGREES));
            if (currentOrientationDriftNorm >= 0.06) {
                continue;
            }
            assertTrue(currentOrientationDriftNorm < 0.06);
            // Check that kalman corrected drift is smaller than normal navigation drift
            assertTrue(currentOrientationDriftNorm < navigationOrientationDrift);
            assertEquals(currentOrientationDriftNorm, kalmanOrientationDrift,
                    ABSOLUTE_ERROR);

            final Angle currentOrientationDriftNorm1 = kalmanDriftEstimator
                    .getCurrentOrientationDriftAngle();
            assertEquals(currentOrientationDriftNorm,
                    currentOrientationDriftNorm1.getValue().doubleValue(), 0.0);
            assertEquals(AngleUnit.RADIANS, currentOrientationDriftNorm1.getUnit());
            final Angle currentOrientationDriftNorm2 = new Angle(1.0,
                    AngleUnit.DEGREES);
            assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftAngle(
                    currentOrientationDriftNorm2));
            assertEquals(currentOrientationDriftNorm1, currentOrientationDriftNorm2);

            final Double currentPositionDriftPerTimeUnit =
                    kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit();
            assertNotNull(currentPositionDriftPerTimeUnit);
            LOGGER.log(Level.INFO, "Current position drift per time unit: " +
                    speedFormatter.format(currentPositionDriftPerTimeUnit,
                            SpeedUnit.METERS_PER_SECOND));
            if (currentPositionDriftPerTimeUnit >= 1.75) {
                continue;
            }
            assertTrue(currentPositionDriftPerTimeUnit < 1.75);
            assertEquals(currentPositionDriftPerTimeUnit,
                    currentPositionDriftNorm / (N_SAMPLES * TIME_INTERVAL_SECONDS),
                    ABSOLUTE_ERROR);

            final Speed currentPositionDriftPerTimeUnit1 =
                    kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed();
            assertEquals(currentPositionDriftPerTimeUnit,
                    currentPositionDriftPerTimeUnit1.getValue().doubleValue(),
                    0.0);
            assertEquals(SpeedUnit.METERS_PER_SECOND,
                    currentPositionDriftPerTimeUnit1.getUnit());
            final Speed currentPositionDriftPerTimeUnit2 = new Speed(1.0,
                    SpeedUnit.KILOMETERS_PER_HOUR);
            assertTrue(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(
                    currentPositionDriftPerTimeUnit2));
            assertEquals(currentPositionDriftPerTimeUnit1,
                    currentPositionDriftPerTimeUnit2);

            final Double currentVelocityDriftPerTimeUnit =
                    kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit();
            assertNotNull(currentVelocityDriftPerTimeUnit);
            LOGGER.log(Level.INFO, "Current velocity drift per time unit: " +
                    accelerationFormatter.format(currentVelocityDriftPerTimeUnit,
                            AccelerationUnit.METERS_PER_SQUARED_SECOND));
            if (currentVelocityDriftPerTimeUnit >= 0.14) {
                continue;
            }
            assertTrue(currentVelocityDriftPerTimeUnit < 0.14);
            assertEquals(currentVelocityDriftPerTimeUnit,
                    currentVelocityDriftNorm / (N_SAMPLES * TIME_INTERVAL_SECONDS),
                    ABSOLUTE_ERROR);

            final Acceleration currentVelocityDriftPerTimeUnit1 =
                    kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration();
            assertEquals(currentVelocityDriftPerTimeUnit,
                    currentVelocityDriftPerTimeUnit1.getValue().doubleValue(),
                    0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                    currentVelocityDriftPerTimeUnit1.getUnit());
            final Acceleration currentVelocityDriftPerTimeUnit2 =
                    new Acceleration(1.0,
                            AccelerationUnit.FEET_PER_SQUARED_SECOND);
            assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                    currentVelocityDriftPerTimeUnit2));
            assertEquals(currentVelocityDriftPerTimeUnit1,
                    currentVelocityDriftPerTimeUnit2);

            final Double currentOrientationDriftPerTimeUnit =
                    kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit();
            assertNotNull(currentOrientationDriftPerTimeUnit);
            LOGGER.log(Level.INFO, "Current orientation drift per time unit: " +
                    angularSpeedFormatter.format(AngularSpeedConverter.convert(
                            currentOrientationDriftPerTimeUnit,
                            AngularSpeedUnit.RADIANS_PER_SECOND,
                            AngularSpeedUnit.DEGREES_PER_SECOND),
                            AngularSpeedUnit.DEGREES_PER_SECOND));
            if (currentOrientationDriftPerTimeUnit >= 1.7e-3) {
                continue;
            }
            assertTrue(currentOrientationDriftPerTimeUnit < 1.7e-3);
            assertEquals(currentOrientationDriftPerTimeUnit,
                    currentOrientationDriftNorm / (N_SAMPLES * TIME_INTERVAL_SECONDS),
                    ABSOLUTE_ERROR);

            final AngularSpeed currentOrientationDriftPerTimeUnit1 =
                    kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed();
            assertEquals(currentOrientationDriftPerTimeUnit,
                    currentOrientationDriftPerTimeUnit1.getValue().doubleValue(),
                    0.0);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND,
                    currentOrientationDriftPerTimeUnit1.getUnit());
            final AngularSpeed currentOrientationDriftPerTimeUnit2 =
                    new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                    currentOrientationDriftPerTimeUnit2));
            assertEquals(currentOrientationDriftPerTimeUnit1,
                    currentOrientationDriftPerTimeUnit2);

            // reset
            kalmanDriftEstimator.reset();

            assertEquals(mReset, 1);
            assertEquals(0, kalmanDriftEstimator.getNumberOfProcessedSamples());
            assertNull(kalmanDriftEstimator.getCurrentPositionDrift());
            assertFalse(kalmanDriftEstimator.getCurrentPositionDrift(null));
            assertNull(kalmanDriftEstimator.getCurrentVelocityDrift());
            assertFalse(kalmanDriftEstimator.getCurrentVelocityDrift(null));
            assertNull(kalmanDriftEstimator.getCurrentOrientationDrift());
            assertFalse(kalmanDriftEstimator.getCurrentOrientationDrift(null));
            assertNull(kalmanDriftEstimator.getCurrentPositionDriftNormMeters());
            assertNull(kalmanDriftEstimator.getCurrentPositionDriftNorm());
            assertFalse(kalmanDriftEstimator.getCurrentPositionDriftNorm(null));
            assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNormMetersPerSecond());
            assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNorm());
            assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftNorm(null));
            assertNull(kalmanDriftEstimator.getCurrentOrientationDriftRadians());
            assertNull(kalmanDriftEstimator.getCurrentOrientationDriftAngle());
            assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftAngle(null));
            assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit());
            assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
            assertFalse(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(
                    null));
            assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit());
            assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
            assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                    null));
            assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit());
            assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
            assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                    null));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testAddBodyKinematicsAndResetApproximateCalibrationAndEstimatedKalmanConfig()
            throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException, LockedException,
            InertialNavigatorException, DriftEstimationException,
            NotReadyException, InvalidRotationMatrixException,
            RotationException, INSException,
            IntervalDetectorThresholdFactorOptimizerException,
            RandomWalkEstimationException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final Matrix ba = generateBa();
            final Matrix ma = generateMaCommonAxis();
            final Matrix bg = generateBg();
            final Matrix mg = generateMg();
            final Matrix gg =  new Matrix(3, 3);

            final double accelNoiseRootPSD = getAccelNoiseRootPSD();
            final double gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final double accelQuantLevel = 0.0;
            final double gyroQuantLevel = 0.0;

            final NEDFrame nedFrame = generateFrame();
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame);

            final NEDPosition nedPosition = nedFrame.getPosition();
            final CoordinateTransformation nedC = nedFrame.getCoordinateTransformation();

            final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();
            final CoordinateTransformation ecefC = ecefFrame.getCoordinateTransformation();

            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator();
            final EasyGyroscopeCalibrator gyroscopeCalibrator =
                    new EasyGyroscopeCalibrator();

            final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                    new AccelerometerAndGyroscopeMeasurementsGenerator();

            // initialize a threshold optimizer to attempt calibration by generating
            // timed body kinematics with noise, and attempting to find the best
            // threshold to find optimal calibration
            final BracketedAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                    buildOptimizer(ba, ma, accelNoiseRootPSD, gyroNoiseRootPSD,
                            accelerometerCalibrator, gyroscopeCalibrator, new NEDFrame(nedFrame),
                            new ECEFFrame(ecefFrame),
                            generator);
            final double thresholdFactor = optimizer.optimize();

            assertEquals(thresholdFactor, optimizer.getOptimalThresholdFactor(),
                    0.0);

            final Matrix estimatedBa = Matrix.newFromArray(
                    optimizer.getEstimatedAccelerometerBiases());
            final Matrix estimatedMa = optimizer.getEstimatedAccelerometerMa();

            final Matrix estimatedBg = Matrix.newFromArray(
                    optimizer.getEstimatedGyroscopeBiases());
            final Matrix estimatedMg = optimizer.getEstimatedGyroscopeMg();
            final Matrix estimatedGg = optimizer.getEstimatedGyroscopeGg();

            // clear all previously generated data
            mTimedBodyKinematics.clear();

            final RandomWalkEstimator randomWalkEstimator = new RandomWalkEstimator();
            randomWalkEstimator.setNedPositionAndNedOrientation(nedPosition, nedC);
            randomWalkEstimator.setAccelerationBias(estimatedBa);
            randomWalkEstimator.setAccelerationCrossCouplingErrors(estimatedMa);
            randomWalkEstimator.setAngularSpeedBias(estimatedBg);
            randomWalkEstimator.setAngularSpeedCrossCouplingErrors(estimatedMg);
            randomWalkEstimator.setAngularSpeedGDependantCrossBias(estimatedGg);
            randomWalkEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

            final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                    gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);
            final Random random = new Random();

            // this is the true body kinematics that should be measured on a perfect
            // sensor if there were no noise or calibration errors at current position
            // and orientations
            final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                            ecefC, ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition);

            final BodyKinematics measuredKinematics = new BodyKinematics();
            for (int i = 0; i < N_SAMPLES; i++) {
                BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                        trueKinematics, errors, random, measuredKinematics);

                randomWalkEstimator.addBodyKinematics(measuredKinematics);

                assertEquals(randomWalkEstimator.getNumberOfProcessedSamples(), i + 1);
                assertFalse(randomWalkEstimator.isRunning());
            }


            final INSLooselyCoupledKalmanConfigCreator kalmanConfigCreator =
                    new INSLooselyCoupledKalmanConfigCreator(generator,
                            randomWalkEstimator);
            final INSLooselyCoupledKalmanConfig kalmanConfig = kalmanConfigCreator.create();

            final INSLooselyCoupledKalmanInitializerConfigCreator initConfigCreator =
                    new INSLooselyCoupledKalmanInitializerConfigCreator(
                            accelerometerCalibrator, gyroscopeCalibrator,
                            randomWalkEstimator);
            final INSLooselyCoupledKalmanInitializerConfig initConfig = initConfigCreator.create();

            // use real calibration values to generate measurements with errors
            final ECEFFrame navigationFrame = new ECEFFrame(ecefFrame);
            final BodyKinematicsFixer fixer = new BodyKinematicsFixer();
            // use estimated calibration data to fix measurements
            fixer.setAccelerationBias(estimatedBa);
            fixer.setAccelerationCrossCouplingErrors(estimatedMa);
            fixer.setAngularSpeedBias(estimatedBg);
            fixer.setAngularSpeedCrossCouplingErrors(estimatedMg);
            fixer.setAngularSpeedGDependantCrossBias(estimatedGg);

            final KalmanDriftEstimator kalmanDriftEstimator = new KalmanDriftEstimator(
                    nedFrame, estimatedBa, estimatedMa, estimatedBg,
                    estimatedMg, estimatedGg, kalmanConfig, initConfig,
                    this);
            kalmanDriftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

            final DriftEstimator driftEstimator = new DriftEstimator(nedFrame,
                    estimatedBa, estimatedMa, estimatedBg, estimatedMg,
                    estimatedGg);
            driftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

            final INSLooselyCoupledKalmanFilteredEstimator kalmanEstimator =
                    new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig,
                            initConfig, new ECEFFrame(ecefFrame));

            reset();
            assertEquals(mStart, 0);
            assertEquals(mBodyKinematicsAdded, 0);
            assertEquals(mReset, 0);
            assertEquals(kalmanDriftEstimator.getNumberOfProcessedSamples(), 0);
            assertFalse(kalmanDriftEstimator.isRunning());
            assertTrue(kalmanDriftEstimator.isFixKinematicsEnabled());

            final BodyKinematics fixedKinematics = new BodyKinematics();
            for (int i = 0; i < N_SAMPLES; i++) {
                BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                        trueKinematics, errors, random, measuredKinematics);

                kalmanDriftEstimator.addBodyKinematics(measuredKinematics);
                driftEstimator.addBodyKinematics(measuredKinematics);

                assertEquals(kalmanDriftEstimator.getNumberOfProcessedSamples(), i + 1);
                assertFalse(kalmanDriftEstimator.isRunning());

                fixer.fix(measuredKinematics, fixedKinematics);
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                        navigationFrame, fixedKinematics, navigationFrame);

                kalmanEstimator.update(fixedKinematics, i * TIME_INTERVAL_SECONDS);
            }

            assertEquals(kalmanDriftEstimator.getNumberOfProcessedSamples(), N_SAMPLES);
            assertFalse(kalmanDriftEstimator.isRunning());
            assertEquals(mStart, 1);
            assertEquals(mBodyKinematicsAdded, N_SAMPLES);
            assertEquals(mReset, 0);

            final INSLooselyCoupledKalmanState kalmanState = kalmanEstimator.getState();

            final double navigationPositionDrift = positionDrift(
                    ecefFrame, navigationFrame);
            final double navigationVelocityDrift = velocityDrift(
                    ecefFrame, navigationFrame);
            final double navigationOrientationDrift = orientationDrift(
                    ecefFrame, navigationFrame);

            final double kalmanPositionDrift = positionDrift(
                    ecefFrame, kalmanState);
            final double kalmanVelocityDrift = velocityDrift(
                    ecefFrame, kalmanState);
            final double kalmanOrientationDrift = orientationDrift(
                    ecefFrame, kalmanState);

            final ECEFPosition currentPositionDrift1 = kalmanDriftEstimator
                    .getCurrentPositionDrift();
            final ECEFPosition currentPositionDrift2 = new ECEFPosition();
            assertTrue(kalmanDriftEstimator.getCurrentPositionDrift(currentPositionDrift2));
            assertEquals(currentPositionDrift1, currentPositionDrift2);

            final ECEFVelocity currentVelocityDrift1 = kalmanDriftEstimator
                    .getCurrentVelocityDrift();
            final ECEFVelocity currentVelocityDrift2 = new ECEFVelocity();
            assertTrue(kalmanDriftEstimator.getCurrentVelocityDrift(currentVelocityDrift2));
            assertEquals(currentVelocityDrift1, currentVelocityDrift2);

            final Rotation3D currentOrientationDrift1 = kalmanDriftEstimator
                    .getCurrentOrientationDrift();
            final Quaternion currentOrientationDrift2 = new Quaternion();
            assertTrue(kalmanDriftEstimator.getCurrentOrientationDrift(currentOrientationDrift2));
            assertEquals(currentOrientationDrift1, currentOrientationDrift2);

            final DistanceFormatter distanceFormatter = new DistanceFormatter();
            final SpeedFormatter speedFormatter = new SpeedFormatter();
            final AngleFormatter angleFormatter = new AngleFormatter();
            final AccelerationFormatter accelerationFormatter =
                    new AccelerationFormatter();
            final AngularSpeedFormatter angularSpeedFormatter =
                    new AngularSpeedFormatter();

            final Double currentPositionDriftNorm = kalmanDriftEstimator
                    .getCurrentPositionDriftNormMeters();
            assertNotNull(currentPositionDriftNorm);
            assertEquals(currentPositionDriftNorm, currentPositionDrift1.getNorm(),
                    0.0);
            LOGGER.log(Level.INFO, "Current position drift: " +
                    distanceFormatter.format(currentPositionDriftNorm,
                            DistanceUnit.METER));
            if (currentPositionDriftNorm >= 92.0) {
                continue;
            }
            assertTrue(currentPositionDriftNorm < 92.0);
            // Check that kalman corrected drift is smaller than normal navigation drift
            assertTrue(currentPositionDriftNorm < navigationPositionDrift);
            assertEquals(currentPositionDriftNorm, kalmanPositionDrift,
                    ABSOLUTE_ERROR);

            final Distance currentPositionDriftNorm1 = kalmanDriftEstimator
                    .getCurrentPositionDriftNorm();
            assertEquals(currentPositionDriftNorm,
                    currentPositionDriftNorm1.getValue().doubleValue(), 0.0);
            assertEquals(DistanceUnit.METER, currentPositionDriftNorm1.getUnit());
            final Distance currentPositionDriftNorm2 = new Distance(1.0,
                    DistanceUnit.FOOT);
            assertTrue(kalmanDriftEstimator.getCurrentPositionDriftNorm(
                    currentPositionDriftNorm2));
            assertEquals(currentPositionDriftNorm1, currentPositionDriftNorm2);

            final Double currentVelocityDriftNorm = kalmanDriftEstimator
                    .getCurrentVelocityDriftNormMetersPerSecond();
            assertNotNull(currentVelocityDriftNorm);
            assertEquals(currentVelocityDriftNorm, currentVelocityDrift1.getNorm(),
                    0.0);
            LOGGER.log(Level.INFO, "Current velocity drift: " +
                    speedFormatter.format(currentVelocityDriftNorm,
                            SpeedUnit.METERS_PER_SECOND));
            if (currentVelocityDriftNorm >= 6.0) {
                continue;
            }
            assertTrue(currentVelocityDriftNorm < 6.0);
            // Check that kalman corrected drift is smaller than normal navigation drift
            assertTrue(currentVelocityDriftNorm < navigationVelocityDrift);
            assertEquals(currentVelocityDriftNorm, kalmanVelocityDrift,
                    ABSOLUTE_ERROR);

            final Speed currentVelocityDriftNorm1 = kalmanDriftEstimator
                    .getCurrentVelocityDriftNorm();
            assertEquals(currentVelocityDriftNorm,
                    currentVelocityDriftNorm1.getValue().doubleValue(), 0.0);
            assertEquals(SpeedUnit.METERS_PER_SECOND,
                    currentVelocityDriftNorm1.getUnit());
            final Speed currentVelocityDriftNorm2 = new Speed(1.0,
                    SpeedUnit.KILOMETERS_PER_HOUR);
            assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftNorm(
                    currentVelocityDriftNorm2));
            assertEquals(currentVelocityDriftNorm1, currentVelocityDriftNorm2);

            final Double currentOrientationDriftNorm = kalmanDriftEstimator
                    .getCurrentOrientationDriftRadians();
            assertNotNull(currentOrientationDriftNorm);
            assertEquals(currentOrientationDriftNorm,
                    currentOrientationDrift1.getRotationAngle(), 0.0);
            LOGGER.log(Level.INFO, "Current orientation drift: " +
                    angleFormatter.format(AngleConverter.convert(
                            currentOrientationDriftNorm, AngleUnit.RADIANS,
                            AngleUnit.DEGREES), AngleUnit.DEGREES));
            if (currentOrientationDriftNorm >= 0.06) {
                continue;
            }
            assertTrue(currentOrientationDriftNorm < 0.06);
            // Check that kalman corrected drift is smaller than normal navigation drift
            assertTrue(currentOrientationDriftNorm < navigationOrientationDrift);
            assertEquals(currentOrientationDriftNorm, kalmanOrientationDrift,
                    ABSOLUTE_ERROR);

            final Angle currentOrientationDriftNorm1 = kalmanDriftEstimator
                    .getCurrentOrientationDriftAngle();
            assertEquals(currentOrientationDriftNorm,
                    currentOrientationDriftNorm1.getValue().doubleValue(), 0.0);
            assertEquals(AngleUnit.RADIANS, currentOrientationDriftNorm1.getUnit());
            final Angle currentOrientationDriftNorm2 = new Angle(1.0,
                    AngleUnit.DEGREES);
            assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftAngle(
                    currentOrientationDriftNorm2));
            assertEquals(currentOrientationDriftNorm1, currentOrientationDriftNorm2);

            final Double currentPositionDriftPerTimeUnit =
                    kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit();
            assertNotNull(currentPositionDriftPerTimeUnit);
            LOGGER.log(Level.INFO, "Current position drift per time unit: " +
                    speedFormatter.format(currentPositionDriftPerTimeUnit,
                            SpeedUnit.METERS_PER_SECOND));
            if (currentPositionDriftPerTimeUnit >= 1.75) {
                continue;
            }
            assertTrue(currentPositionDriftPerTimeUnit < 1.75);
            assertEquals(currentPositionDriftPerTimeUnit,
                    currentPositionDriftNorm / (N_SAMPLES * TIME_INTERVAL_SECONDS),
                    ABSOLUTE_ERROR);

            final Speed currentPositionDriftPerTimeUnit1 =
                    kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed();
            assertEquals(currentPositionDriftPerTimeUnit,
                    currentPositionDriftPerTimeUnit1.getValue().doubleValue(),
                    0.0);
            assertEquals(SpeedUnit.METERS_PER_SECOND,
                    currentPositionDriftPerTimeUnit1.getUnit());
            final Speed currentPositionDriftPerTimeUnit2 = new Speed(1.0,
                    SpeedUnit.KILOMETERS_PER_HOUR);
            assertTrue(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(
                    currentPositionDriftPerTimeUnit2));
            assertEquals(currentPositionDriftPerTimeUnit1,
                    currentPositionDriftPerTimeUnit2);

            final Double currentVelocityDriftPerTimeUnit =
                    kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit();
            assertNotNull(currentVelocityDriftPerTimeUnit);
            LOGGER.log(Level.INFO, "Current velocity drift per time unit: " +
                    accelerationFormatter.format(currentVelocityDriftPerTimeUnit,
                            AccelerationUnit.METERS_PER_SQUARED_SECOND));
            if (currentVelocityDriftPerTimeUnit >= 0.14) {
                continue;
            }
            assertTrue(currentVelocityDriftPerTimeUnit < 0.14);
            assertEquals(currentVelocityDriftPerTimeUnit,
                    currentVelocityDriftNorm / (N_SAMPLES * TIME_INTERVAL_SECONDS),
                    ABSOLUTE_ERROR);

            final Acceleration currentVelocityDriftPerTimeUnit1 =
                    kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration();
            assertEquals(currentVelocityDriftPerTimeUnit,
                    currentVelocityDriftPerTimeUnit1.getValue().doubleValue(),
                    0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND,
                    currentVelocityDriftPerTimeUnit1.getUnit());
            final Acceleration currentVelocityDriftPerTimeUnit2 =
                    new Acceleration(1.0,
                            AccelerationUnit.FEET_PER_SQUARED_SECOND);
            assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                    currentVelocityDriftPerTimeUnit2));
            assertEquals(currentVelocityDriftPerTimeUnit1,
                    currentVelocityDriftPerTimeUnit2);

            final Double currentOrientationDriftPerTimeUnit =
                    kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit();
            assertNotNull(currentOrientationDriftPerTimeUnit);
            LOGGER.log(Level.INFO, "Current orientation drift per time unit: " +
                    angularSpeedFormatter.format(AngularSpeedConverter.convert(
                            currentOrientationDriftPerTimeUnit,
                            AngularSpeedUnit.RADIANS_PER_SECOND,
                            AngularSpeedUnit.DEGREES_PER_SECOND),
                            AngularSpeedUnit.DEGREES_PER_SECOND));
            if (currentOrientationDriftPerTimeUnit >= 1.7e-3) {
                continue;
            }
            assertTrue(currentOrientationDriftPerTimeUnit < 1.7e-3);
            assertEquals(currentOrientationDriftPerTimeUnit,
                    currentOrientationDriftNorm / (N_SAMPLES * TIME_INTERVAL_SECONDS),
                    ABSOLUTE_ERROR);

            final AngularSpeed currentOrientationDriftPerTimeUnit1 =
                    kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed();
            assertEquals(currentOrientationDriftPerTimeUnit,
                    currentOrientationDriftPerTimeUnit1.getValue().doubleValue(),
                    0.0);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND,
                    currentOrientationDriftPerTimeUnit1.getUnit());
            final AngularSpeed currentOrientationDriftPerTimeUnit2 =
                    new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                    currentOrientationDriftPerTimeUnit2));
            assertEquals(currentOrientationDriftPerTimeUnit1,
                    currentOrientationDriftPerTimeUnit2);

            // reset
            kalmanDriftEstimator.reset();

            assertEquals(mReset, 1);
            assertEquals(0, kalmanDriftEstimator.getNumberOfProcessedSamples());
            assertNull(kalmanDriftEstimator.getCurrentPositionDrift());
            assertFalse(kalmanDriftEstimator.getCurrentPositionDrift(null));
            assertNull(kalmanDriftEstimator.getCurrentVelocityDrift());
            assertFalse(kalmanDriftEstimator.getCurrentVelocityDrift(null));
            assertNull(kalmanDriftEstimator.getCurrentOrientationDrift());
            assertFalse(kalmanDriftEstimator.getCurrentOrientationDrift(null));
            assertNull(kalmanDriftEstimator.getCurrentPositionDriftNormMeters());
            assertNull(kalmanDriftEstimator.getCurrentPositionDriftNorm());
            assertFalse(kalmanDriftEstimator.getCurrentPositionDriftNorm(null));
            assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNormMetersPerSecond());
            assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNorm());
            assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftNorm(null));
            assertNull(kalmanDriftEstimator.getCurrentOrientationDriftRadians());
            assertNull(kalmanDriftEstimator.getCurrentOrientationDriftAngle());
            assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftAngle(null));
            assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit());
            assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
            assertFalse(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(
                    null));
            assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit());
            assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
            assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                    null));
            assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit());
            assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
            assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                    null));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onStart(final DriftEstimator estimator) {
        checkLocked((KalmanDriftEstimator) estimator);
        mStart++;
    }

    @Override
    public void onBodyKinematicsAdded(
            final DriftEstimator estimator,
            final BodyKinematics measuredKinematics,
            final BodyKinematics fixedKinematics) {
        if (mBodyKinematicsAdded == 0) {
            checkLocked((KalmanDriftEstimator) estimator);
        }
        mBodyKinematicsAdded++;
    }

    @Override
    public void onReset(final DriftEstimator estimator) {
        checkLocked((KalmanDriftEstimator) estimator);
        mReset++;
    }

    private void reset() {
        mStart = 0;
        mBodyKinematicsAdded = 0;
        mReset = 0;
    }

    private void checkLocked(final KalmanDriftEstimator estimator) {
        assertTrue(estimator.isRunning());
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setReferenceFrame(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setReferenceNedFrame(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setReferenceEcefPosition(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setReferenceEcefVelocity(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setReferenceEcefCoordinateTransformation(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setReferenceNedPosition(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setReferenceNedVelocity(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setReferenceNedCoordinateTransformation(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final InvalidSourceAndDestinationFrameTypeException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAccelerationBias((Matrix) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAccelerationBias((double[]) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAccelerationBias((AccelerationTriad) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAccelerationBiasX(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAccelerationBiasY(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAccelerationBiasZ(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAccelerationBias(0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAccelerationBiasX(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAccelerationBiasY(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAccelerationBiasZ(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAccelerationBias(null, null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAccelerationCrossCouplingErrors(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAccelerationSx(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAccelerationSy(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAccelerationSz(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAccelerationMxy(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAccelerationMxz(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAccelerationMyx(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAccelerationMyz(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAccelerationMzx(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAccelerationMzy(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAccelerationScalingFactors(0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAccelerationCrossCouplingErrors(
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAccelerationScalingFactorsAndCrossCouplingErrors(
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAngularSpeedBias((Matrix) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAngularSpeedBias((double[]) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAngularSpeedBias((AngularSpeedTriad) null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAngularSpeedBiasX(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAngularSpeedBiasY(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAngularSpeedBiasZ(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAngularSpeedBias(0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAngularSpeedBiasX(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAngularSpeedBiasY(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAngularSpeedBiasZ(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAngularSpeedBias(null, null, null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setAngularSpeedCrossCouplingErrors(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAngularSpeedSx(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAngularSpeedSy(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAngularSpeedSz(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAngularSpeedMxy(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAngularSpeedMxz(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAngularSpeedMyx(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAngularSpeedMyz(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAngularSpeedMzx(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAngularSpeedMzy(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAngularSpeedScalingFactors(0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAngularSpeedCrossCouplingErrors(
                    0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAngularSpeedScalingFactorsAndCrossCouplingErrors(
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final AlgebraException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.setAngularSpeedGDependantCrossBias(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setFixKinematicsEnabled(false);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setTimeInterval(0.0);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setTimeInterval(new Time(1.0, TimeUnit.SECOND));
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setKalmanConfig(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
        try {
            estimator.setInitConfig(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
    }

    private BracketedAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer buildOptimizer(
            final Matrix ba, final Matrix ma,
            final double accelNoiseRootPSD,
            final double gyroNoiseRootPSD,
            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator,
            final EasyGyroscopeCalibrator gyroscopeCalibrator)
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, WrongSizeException, LockedException {

        final NEDFrame nedFrame = generateFrame();
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                new AccelerometerAndGyroscopeMeasurementsGenerator();

        return buildOptimizer(ba, ma, accelNoiseRootPSD, gyroNoiseRootPSD,
                accelerometerCalibrator, gyroscopeCalibrator, nedFrame, ecefFrame,
                generator);
    }

    private BracketedAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer buildOptimizer(
            final Matrix ba, final Matrix ma,
            final double accelNoiseRootPSD,
            final double gyroNoiseRootPSD,
            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator,
            final EasyGyroscopeCalibrator gyroscopeCalibrator,
            final NEDFrame nedFrame, final ECEFFrame ecefFrame,
            AccelerometerAndGyroscopeMeasurementsGenerator generator)
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, WrongSizeException, LockedException {

        mTimedBodyKinematics.clear();

        // generate measurements

        final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
        final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
        generateBodyKinematics(nedFrame, ecefFrame, false, ma,
                accelNoiseRootPSD, gyroNoiseRootPSD, numSequences,
                numMeasurements);

        // we only use the generator at this point to get an estimated average of
        // initial gyroscope bias (but we could skip this as well and probably get
        // a similar gyroscope calibration accuracy).

        for (TimedBodyKinematics timedBodyKinematics : mTimedBodyKinematics) {
            assertTrue(generator.process(timedBodyKinematics));
        }

        // as an initial value for gyroscope bias we can use the average
        // gyroscope values during initialization. A more accurate initial
        // guess for bias could be obtained by using leveling with magnetometer
        // and accelerometer readings (once both magnetometer and accelerometer
        // are calibrated).
        final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
        final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

        final ECEFGravity gravity = ECEFGravityEstimator
                .estimateGravityAndReturnNew(ecefFrame);

        // configure calibrators and data source
        final Matrix initialBa = new Matrix(3, 1);
        final Matrix initialMa = new Matrix(3, 3);
        accelerometerCalibrator.setGroundTruthGravityNorm(gravity.getNorm());
        accelerometerCalibrator.setCommonAxisUsed(true);
        accelerometerCalibrator.setInitialBias(initialBa);
        accelerometerCalibrator.setInitialMa(initialMa);

        final Matrix initialMg = new Matrix(3, 3);
        final Matrix initialGg = new Matrix(3, 3);
        gyroscopeCalibrator.setCommonAxisUsed(true);
        gyroscopeCalibrator.setGDependentCrossBiasesEstimated(false);
        gyroscopeCalibrator.setInitialBias(initialBg);
        gyroscopeCalibrator.setInitialMg(initialMg);
        gyroscopeCalibrator.setInitialGg(initialGg);
        gyroscopeCalibrator.setAccelerometerBias(ba);
        gyroscopeCalibrator.setAccelerometerMa(ma);

        // create optimizer
        return new BracketedAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(
                mDataSource, accelerometerCalibrator, gyroscopeCalibrator);
    }

    @SuppressWarnings("SameParameterValue")
    private void generateBodyKinematics(
            final NEDFrame nedFrame, final ECEFFrame ecefFrame,
            final boolean changePosition, final Matrix ma, final double accelNoiseRootPSD,
            final double gyroNoiseRootPSD, final int numSequences,
            final int numMeasurements) throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD,
                gyroNoiseRootPSD, accelQuantLevel, gyroQuantLevel);

        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                        ecefFrame, ecefFrame);

        // generate initial static samples
        final int initialStaticSamples = TriadStaticIntervalDetector
                .DEFAULT_INITIAL_STATIC_SAMPLES;
        generateStaticSamples(initialStaticSamples, trueKinematics, errors, random,
                0);

        final int n = Math.max(numSequences + 1, numMeasurements);

        final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
        final int dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

        int start = initialStaticSamples;
        for (int i = 0; i < n; i++) {
            // generate static samples
            generateStaticSamples(staticPeriodLength, trueKinematics, errors, random,
                    start);
            start += staticPeriodLength;

            // generate dynamic samples
            generateDynamicSamples(dynamicPeriodLength, trueKinematics,
                    randomizer, ecefFrame, nedFrame, errors, random, start,
                    changePosition);
            start += dynamicPeriodLength;
        }
    }

    private NEDFrame generateFrame() throws InvalidSourceAndDestinationFrameTypeException {
        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final CoordinateTransformation nedC = new CoordinateTransformation(
                roll, pitch, yaw, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        return new NEDFrame(nedPosition, nedC);
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

    private Matrix generateMaCommonAxis() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                0.0, -600e-6, 250e-6,
                0.0, 0.0, 450e-6
        }, false);

        return result;
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

    private void generateStaticSamples(
            final int numSamples,
            final BodyKinematics trueKinematics,
            final IMUErrors errors,
            final Random random,
            final int startSample) {

        for (int i = 0, j = startSample; i < numSamples; i++, j++) {

            final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                    .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                            random);

            final TimedBodyKinematics timedMeasuredKinematics = new TimedBodyKinematics();
            timedMeasuredKinematics.setKinematics(measuredKinematics);
            timedMeasuredKinematics.setTimestampSeconds(
                    j * TIME_INTERVAL_SECONDS);

            mTimedBodyKinematics.add(timedMeasuredKinematics);
        }
    }

    @SuppressWarnings("SameParameterValue")
    private void generateDynamicSamples(
            final int numSamples,
            final BodyKinematics trueKinematics,
            final UniformRandomizer randomizer,
            final ECEFFrame ecefFrame,
            final NEDFrame nedFrame,
            final IMUErrors errors,
            final Random random,
            final int startSample,
            final boolean changePosition)
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
        final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
        final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

        final double deltaX = changePosition ?
                randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;
        final double deltaY = changePosition ?
                randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;
        final double deltaZ = changePosition ?
                randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;

        final double deltaRoll = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaPitch = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaYaw = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final double ecefX = ecefFrame.getX();
        final double ecefY = ecefFrame.getY();
        final double ecefZ = ecefFrame.getZ();

        final CoordinateTransformation nedC = nedFrame.getCoordinateTransformation();

        final double roll = nedC.getRollEulerAngle();
        final double pitch = nedC.getPitchEulerAngle();
        final double yaw = nedC.getYawEulerAngle();

        final Quaternion beforeQ = new Quaternion();
        nedC.asRotation(beforeQ);

        NEDFrame oldNedFrame = new NEDFrame(nedFrame);
        NEDFrame newNedFrame = new NEDFrame();
        ECEFFrame oldEcefFrame = new ECEFFrame(ecefFrame);
        ECEFFrame newEcefFrame = new ECEFFrame();

        double oldEcefX = ecefX - deltaX;
        double oldEcefY = ecefY - deltaY;
        double oldEcefZ = ecefZ - deltaZ;
        double oldRoll = roll - deltaRoll;
        double oldPitch = pitch - deltaPitch;
        double oldYaw = yaw - deltaYaw;

        final BodyKinematics measuredBeforeGravityKinematics = BodyKinematicsGenerator
                .generate(TIME_INTERVAL_SECONDS,
                        trueKinematics, errors, random);
        final double beforeMeanFx = measuredBeforeGravityKinematics.getFx();
        final double beforeMeanFy = measuredBeforeGravityKinematics.getFy();
        final double beforeMeanFz = measuredBeforeGravityKinematics.getFz();

        final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                new BodyKinematicsSequence<>();
        sequence.setBeforeMeanSpecificForceCoordinates(
                beforeMeanFx, beforeMeanFy, beforeMeanFz);

        final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                new BodyKinematicsSequence<>();
        final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList =
                new ArrayList<>();

        for (int i = 0, j = startSample; i < numSamples; i++, j++) {
            final double progress = (double) i / (double) numSamples;

            final double newRoll = oldRoll + interpolate(deltaRoll, progress);
            final double newPitch = oldPitch + interpolate(deltaPitch, progress);
            final double newYaw = oldYaw + interpolate(deltaYaw, progress);
            final CoordinateTransformation newNedC =
                    new CoordinateTransformation(
                            newRoll, newPitch, newYaw,
                            FrameType.BODY_FRAME,
                            FrameType.LOCAL_NAVIGATION_FRAME);
            final NEDPosition newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final double newEcefX = oldEcefX + interpolate(deltaX, progress);
            final double newEcefY = oldEcefY + interpolate(deltaY, progress);
            final double newEcefZ = oldEcefZ + interpolate(deltaZ, progress);

            newEcefFrame.setCoordinates(newEcefX, newEcefY, newEcefZ);

            ECEFtoNEDFrameConverter.convertECEFtoNED(newEcefFrame, newNedFrame);

            final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

            // update true kinematics using new position and rotation
            ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS,
                    newEcefFrame, oldEcefFrame, trueKinematics);

            // add error to true kinematics
            final BodyKinematics measuredKinematics = BodyKinematicsGenerator
                    .generate(TIME_INTERVAL_SECONDS, trueKinematics, errors,
                            random);

            final TimedBodyKinematics timedMeasuredKinematics = new TimedBodyKinematics();
            timedMeasuredKinematics.setKinematics(measuredKinematics);
            timedMeasuredKinematics.setTimestampSeconds(timestampSeconds);

            mTimedBodyKinematics.add(timedMeasuredKinematics);

            final StandardDeviationTimedBodyKinematics trueTimedKinematics =
                    new StandardDeviationTimedBodyKinematics(
                            new BodyKinematics(trueKinematics), timestampSeconds,
                            specificForceStandardDeviation,
                            angularRateStandardDeviation);
            trueTimedKinematicsList.add(trueTimedKinematics);

            oldNedFrame.copyFrom(newNedFrame);
            oldEcefFrame.copyFrom(newEcefFrame);
            oldRoll = newRoll;
            oldPitch = newPitch;
            oldYaw = newYaw;
            oldEcefX = newEcefX;
            oldEcefY = newEcefY;
            oldEcefZ = newEcefZ;
        }

        trueSequence.setItems(trueTimedKinematicsList);

        final Quaternion afterQ = new Quaternion();
        QuaternionIntegrator.integrateGyroSequence(
                trueSequence, beforeQ, afterQ);

        final CoordinateTransformation newNedC =
                new CoordinateTransformation(
                        afterQ.asInhomogeneousMatrix(),
                        FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME);
        newNedFrame.setCoordinateTransformation(newNedC);

        NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);


        // update current ECEF and NED frames
        ecefFrame.copyFrom(newEcefFrame);
        nedFrame.copyFrom(newNedFrame);

        // after dynamic sequence finishes, update true kinematics for a
        // static sequence at current frame
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS,
                newEcefFrame, newEcefFrame, trueKinematics);
    }

    // This is required to simulate a smooth transition of values during
    // dynamic period, to avoid a sudden rotation or translation and simulate
    // a more natural behaviour.
    private double interpolate(final double value, final double progress) {
        return -2.0 * (Math.abs(progress - 0.5) - 0.5) * value;
    }

    private double positionDrift(
            final ECEFFrame frame1, final ECEFFrame frame2) {
        final double x1 = frame1.getX();
        final double y1 = frame1.getY();
        final double z1 = frame1.getZ();

        final double x2 = frame2.getX();
        final double y2 = frame2.getY();
        final double z2 = frame2.getZ();

        final double diffX = x2 - x1;
        final double diffY = y2 - y1;
        final double diffZ = z2 - z1;

        final ECEFPosition position = new ECEFPosition(diffX, diffY, diffZ);
        return position.getNorm();
    }

    private double velocityDrift(
            final ECEFFrame frame1, final ECEFFrame frame2) {
        final double vx1 = frame1.getVx();
        final double vy1 = frame1.getVy();
        final double vz1 = frame1.getVz();

        final double vx2 = frame2.getVx();
        final double vy2 = frame2.getVy();
        final double vz2 = frame2.getVz();

        final double diffVx = vx2 - vx1;
        final double diffVy = vy2 - vy1;
        final double diffVz = vz2 - vz1;

        final ECEFVelocity velocity = new ECEFVelocity(diffVx, diffVy, diffVz);
        return velocity.getNorm();
    }

    private double orientationDrift(
            final ECEFFrame frame1, final ECEFFrame frame2)
            throws InvalidRotationMatrixException {
        final Matrix c1 = frame1.getCoordinateTransformationMatrix();
        final Matrix c2 = frame2.getCoordinateTransformationMatrix();

        final Quaternion q1 = new Quaternion();
        q1.fromMatrix(c1);
        final Quaternion q2 = new Quaternion();
        q2.fromMatrix(c2);

        final Quaternion invQ1 = q1.inverseAndReturnNew();
        final Quaternion diffQ = q2.combineAndReturnNew(invQ1);
        return diffQ.getRotationAngle();
    }

    private double positionDrift(
            final ECEFFrame frame, final INSLooselyCoupledKalmanState state) {
        final double x1 = frame.getX();
        final double y1 = frame.getY();
        final double z1 = frame.getZ();

        final double x2 = state.getX();
        final double y2 = state.getY();
        final double z2 = state.getZ();

        final double diffX = x2 - x1;
        final double diffY = y2 - y1;
        final double diffZ = z2 - z1;

        final ECEFPosition position = new ECEFPosition(diffX, diffY, diffZ);
        return position.getNorm();
    }

    private double velocityDrift(
            final ECEFFrame frame, final INSLooselyCoupledKalmanState state) {
        final double vx1 = frame.getVx();
        final double vy1 = frame.getVy();
        final double vz1 = frame.getVz();

        final double vx2 = state.getVx();
        final double vy2 = state.getVy();
        final double vz2 = state.getVz();

        final double diffVx = vx2 - vx1;
        final double diffVy = vy2 - vy1;
        final double diffVz = vz2 - vz1;

        final ECEFVelocity velocity = new ECEFVelocity(diffVx, diffVy, diffVz);
        return velocity.getNorm();
    }

    private double orientationDrift(
            final ECEFFrame frame, final INSLooselyCoupledKalmanState state)
            throws InvalidRotationMatrixException {
        final Matrix c1 = frame.getCoordinateTransformationMatrix();
        final Matrix c2 = state.getBodyToEcefCoordinateTransformationMatrix();

        final Quaternion q1 = new Quaternion();
        q1.fromMatrix(c1);
        final Quaternion q2 = new Quaternion();
        q2.fromMatrix(c2);

        final Quaternion invQ1 = q1.inverseAndReturnNew();
        final Quaternion diffQ = q2.combineAndReturnNew(invQ1);
        return diffQ.getRotationAngle();
    }

    private INSLooselyCoupledKalmanConfig generateKalmanConfig() {
        // deg^2 per hour converted to rad^2/s
        final double gyroNoisePsd = Math.pow(0.02 * DEG_TO_RAD / 60.0, 2.0);
        //micro-g^2 per Hz converted to m^2*s^-3
        final double accelerometerNoisePsd = Math.pow(200 * MICRO_G_TO_METERS_PER_SECOND_SQUARED, 2.0);
        final double accelerometerBiasPsd = 1e-7; // m^2*s^-5
        final double gyroBiasPsd = 2e-12; // rad^2 * s^-3
        final double positionNoiseSd = 2.5; // m
        final double velocityNoiseSd = 0.1; // m/s

        return new INSLooselyCoupledKalmanConfig(gyroNoisePsd, accelerometerNoisePsd,
                accelerometerBiasPsd, gyroBiasPsd, positionNoiseSd, velocityNoiseSd);
    }

    private INSLooselyCoupledKalmanInitializerConfig generateInitConfig() {
        final double initialAttitudeUncertainty = Math.toRadians(1.0); // 1º
        final double initialVelocityUncertainty = 0.1; // m/s
        final double initialPositionUncertainty = 10.0; //m
        final double initialAccelerationBiasUncertainty = 1000 * MICRO_G_TO_METERS_PER_SECOND_SQUARED;
        final double initialGyroscopeBiasUncertainty = 10.0 * DEG_TO_RAD / 3600.0; // deg/hour converted to rad/sec

        return new INSLooselyCoupledKalmanInitializerConfig(
                initialAttitudeUncertainty, initialVelocityUncertainty,
                initialPositionUncertainty, initialAccelerationBiasUncertainty,
                initialGyroscopeBiasUncertainty);
    }
}
