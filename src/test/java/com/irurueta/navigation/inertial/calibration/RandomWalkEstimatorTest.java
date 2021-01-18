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
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.calibration.bias.BodyKinematicsBiasEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.*;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.*;

public class RandomWalkEstimatorTest implements RandomWalkEstimatorListener {

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

    private static final int N_SAMPLES = 100000;

    private static final int TIMES = 100;

    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-6;

    private int mStart;
    private int mBodyKinematicsAdded;
    private int mReset;

    @Test
    public void testConstructor1() throws WrongSizeException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(new Matrix(3, 1), ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(new double[3], ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, bax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, bay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(new Matrix(3, 1), bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(new double[3], bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad3.getUnit());
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame();
        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1.getCoordinateTransformation();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition1.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC1.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC1, nedC2);

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
    }

    @Test
    public void testConstructor2() throws WrongSizeException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator(this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(new Matrix(3, 1), ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(new double[3], ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, bax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, bay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(new Matrix(3, 1), bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(new double[3], bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad3.getUnit());
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame();
        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1.getCoordinateTransformation();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition1.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC1.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC1, nedC2);

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
    }

    @Test
    public void testConstructor3() throws AlgebraException {
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        RandomWalkEstimator estimator = new RandomWalkEstimator(
                baTriad, ma, bgTriad, mg);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame();
        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1.getCoordinateTransformation();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition1.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC1.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC1, nedC2);

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(baTriad, wrong, bgTriad, mg);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(baTriad, ma, bgTriad, wrong);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(baTriad,
                    new Matrix(1, 3), bgTriad, mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(baTriad,
                    new Matrix(3, 1), bgTriad, mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(baTriad, ma, bgTriad,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(baTriad, ma, bgTriad,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor4() throws AlgebraException {
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        RandomWalkEstimator estimator = new RandomWalkEstimator(
                baTriad, ma, bgTriad, mg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame();
        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1.getCoordinateTransformation();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition1.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC1.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC1, nedC2);

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(baTriad, wrong, bgTriad, mg,
                    this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(baTriad, ma, bgTriad, wrong,
                    this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(baTriad,
                    new Matrix(1, 3), bgTriad, mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(baTriad,
                    new Matrix(3, 1), bgTriad, mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(baTriad, ma, bgTriad,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(baTriad, ma, bgTriad,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor5() throws AlgebraException {
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        RandomWalkEstimator estimator = new RandomWalkEstimator(
                baTriad, ma, bgTriad, mg, gg);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame();
        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1.getCoordinateTransformation();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition1.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC1.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC1, nedC2);

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(baTriad, wrong, bgTriad, mg, gg);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(baTriad, ma, bgTriad, wrong, gg);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(baTriad,
                    new Matrix(1, 3), bgTriad, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(baTriad,
                    new Matrix(3, 1), bgTriad, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(baTriad, ma, bgTriad,
                    new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(baTriad, ma, bgTriad,
                    new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(baTriad, ma, bgTriad, mg,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(baTriad, ma, bgTriad, mg,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor6() throws AlgebraException {
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        RandomWalkEstimator estimator = new RandomWalkEstimator(
                baTriad, ma, bgTriad, mg, gg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame();
        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1.getCoordinateTransformation();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition1.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC1.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC1, nedC2);

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(baTriad, wrong, bgTriad, mg, gg,
                    this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(baTriad, ma, bgTriad, wrong, gg,
                    this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(baTriad,
                    new Matrix(1, 3), bgTriad, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(baTriad,
                    new Matrix(3, 1), bgTriad, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(baTriad, ma, bgTriad,
                    new Matrix(1, 3), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(baTriad, ma, bgTriad,
                    new Matrix(3, 1), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(baTriad, ma, bgTriad, mg,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(baTriad, ma, bgTriad, mg,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor7() throws AlgebraException {
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        RandomWalkEstimator estimator = new RandomWalkEstimator(
                ba, ma, bg, mg);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame();
        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1.getCoordinateTransformation();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition1.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC1.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC1, nedC2);

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(ba, wrong, bg, mg);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma, bg, wrong);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(new Matrix(1, 1),
                    ma, bg, mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(new Matrix(3, 3),
                    ma, bg, mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba,
                    new Matrix(1, 3), bg, mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba,
                    new Matrix(3, 1), bg, mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma,
                    new Matrix(1, 1), mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma,
                    new Matrix(1, 3), mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma, bg,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma, bg,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor8() throws AlgebraException {
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        RandomWalkEstimator estimator = new RandomWalkEstimator(
                ba, ma, bg, mg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame();
        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1.getCoordinateTransformation();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition1.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC1.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC1, nedC2);

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(ba, wrong, bg, mg, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma, bg, wrong, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(new Matrix(1, 1),
                    ma, bg, mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(new Matrix(3, 3),
                    ma, bg, mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba,
                    new Matrix(1, 3), bg, mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba,
                    new Matrix(3, 1), bg, mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma,
                    new Matrix(1, 1), mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma,
                    new Matrix(1, 3), mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma, bg,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma, bg,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor9() throws AlgebraException {
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        RandomWalkEstimator estimator = new RandomWalkEstimator(
                ba, ma, bg, mg, gg);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame();
        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1.getCoordinateTransformation();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition1.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC1.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC1, nedC2);

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(ba, wrong, bg, mg, gg);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma, bg, wrong, gg);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(new Matrix(1, 1),
                    ma, bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(new Matrix(3, 3),
                    ma, bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba,
                    new Matrix(1, 3), bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba,
                    new Matrix(3, 1), bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma,
                    new Matrix(1, 1), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma,
                    new Matrix(1, 3), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma, bg,
                    new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma, bg,
                    new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma, bg, mg,
                    new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma, bg, mg,
                    new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor10() throws AlgebraException {
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();
        RandomWalkEstimator estimator = new RandomWalkEstimator(
                ba, ma, bg, mg, gg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame();
        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1.getCoordinateTransformation();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition1.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC1.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC1, nedC2);

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(ba, wrong, bg, mg, gg,
                    this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma, bg, wrong, gg,
                    this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(new Matrix(1, 1),
                    ma, bg, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(new Matrix(3, 3),
                    ma, bg, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba,
                    new Matrix(1, 3), bg, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba,
                    new Matrix(3, 1), bg, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma,
                    new Matrix(1, 1), mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma,
                    new Matrix(1, 3), mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma, bg,
                    new Matrix(1, 3), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma, bg,
                    new Matrix(3, 1), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma, bg, mg,
                    new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ba, ma, bg, mg,
                    new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor11() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();

        final RandomWalkEstimator estimator = new RandomWalkEstimator(
                nedPosition, nedC);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(new Matrix(3, 1), ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(new double[3], ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, bax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, bay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(new Matrix(3, 1), bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(new double[3], bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad3.getUnit());
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
    }

    @Test
    public void testConstructor12() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();

        final RandomWalkEstimator estimator = new RandomWalkEstimator(
                nedPosition, nedC, this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(new Matrix(3, 1), ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(new double[3], ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, bax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, bay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(new Matrix(3, 1), bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(new double[3], bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad3.getUnit());
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
    }

    @Test
    public void testConstructor13() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                nedPosition, nedC, baTriad, ma, bgTriad, mg);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    baTriad, wrong, bgTriad, mg);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    baTriad, ma, bgTriad, wrong);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    new Matrix(1, 3), bgTriad, mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    new Matrix(3, 1), bgTriad, mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor14() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                nedPosition, nedC, baTriad, ma, bgTriad, mg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    baTriad, wrong, bgTriad, mg, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    baTriad, ma, bgTriad, wrong, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    new Matrix(1, 3), bgTriad, mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    new Matrix(3, 1), bgTriad, mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor15() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                nedPosition, nedC, baTriad, ma, bgTriad, mg, gg);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    baTriad, wrong, bgTriad, mg, gg);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    baTriad, ma, bgTriad, wrong, gg);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    new Matrix(1, 3), bgTriad, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    new Matrix(3, 1), bgTriad, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, mg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, mg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor16() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                nedPosition, nedC, baTriad, ma, bgTriad, mg, gg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    baTriad, wrong, bgTriad, mg, gg, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    baTriad, ma, bgTriad, wrong, gg, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    new Matrix(1, 3), bgTriad, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    new Matrix(3, 1), bgTriad, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(1, 3), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(3, 1), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, mg, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, mg, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor17() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                nedPosition, nedC, ba, ma, bg, mg);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    ba, wrong, bg, mg);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    ba, ma, bg, wrong);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    new Matrix(1, 1), ma, bg, mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    new Matrix(3, 3), ma, bg, mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    new Matrix(1, 3), bg, mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    new Matrix(3, 1), bg, mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, new Matrix(1, 1), mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, new Matrix(3, 3), mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor18() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                nedPosition, nedC, ba, ma, bg, mg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    ba, wrong, bg, mg, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    ba, ma, bg, wrong, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    new Matrix(1, 1), ma, bg, mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    new Matrix(3, 3), ma, bg, mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    new Matrix(1, 3), bg, mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    new Matrix(3, 1), bg, mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, new Matrix(1, 1), mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, new Matrix(3, 3), mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor19() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                nedPosition, nedC, ba, ma, bg, mg, gg);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    ba, wrong, bg, mg, gg);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    ba, ma, bg, wrong, gg);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    new Matrix(1, 1), ma, bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    new Matrix(3, 3), ma, bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    new Matrix(1, 3), bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    new Matrix(3, 1), bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, new Matrix(1, 1), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, new Matrix(3, 3), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, mg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, mg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor20() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                nedPosition, nedC, ba, ma, bg, mg, gg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    ba, wrong, bg, mg, gg, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    ba, ma, bg, wrong, gg, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    new Matrix(1, 1), ma, bg, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    new Matrix(3, 3), ma, bg, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    new Matrix(1, 3), bg, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    new Matrix(3, 1), bg, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, new Matrix(1, 1), mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, new Matrix(3, 3), mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, new Matrix(1, 3), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, new Matrix(3, 1), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, mg, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, mg, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor21() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final RandomWalkEstimator estimator = new RandomWalkEstimator(
                ecefPosition, nedC);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(new Matrix(3, 1), ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(new double[3], ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, bax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, bay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(new Matrix(3, 1), bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(new double[3], bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad3.getUnit());
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
    }

    @Test
    public void testConstructor22() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final RandomWalkEstimator estimator = new RandomWalkEstimator(
                ecefPosition, nedC, this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(new Matrix(3, 1), ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(new double[3], ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, bax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, bay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(new Matrix(3, 1), bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(new double[3], bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad3.getUnit());
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
    }

    @Test
    public void testConstructor23() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                ecefPosition, nedC, baTriad, ma, bgTriad, mg);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    baTriad, wrong, bgTriad, mg);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    baTriad, ma, bgTriad, wrong);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    new Matrix(1, 3), bgTriad, mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    new Matrix(3, 1), bgTriad, mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor24() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                ecefPosition, nedC, baTriad, ma, bgTriad, mg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    baTriad, wrong, bgTriad, mg, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    baTriad, ma, bgTriad, wrong, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    new Matrix(1, 3), bgTriad, mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    new Matrix(3, 1), bgTriad, mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor25() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                ecefPosition, nedC, baTriad, ma, bgTriad, mg, gg);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    baTriad, wrong, bgTriad, mg, gg);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    baTriad, ma, bgTriad, wrong, gg);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    new Matrix(1, 3), bgTriad, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    new Matrix(3, 1), bgTriad, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, mg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, mg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor26() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                ecefPosition, nedC, baTriad, ma, bgTriad, mg, gg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    baTriad, wrong, bgTriad, mg, gg, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    baTriad, ma, bgTriad, wrong, gg, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    new Matrix(1, 3), bgTriad, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    new Matrix(3, 1), bgTriad, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(1, 3), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(3, 1), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, mg, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, mg, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor27() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                ecefPosition, nedC, ba, ma, bg, mg);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    ba, wrong, bg, mg);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    ba, ma, bg, wrong);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    new Matrix(1, 1), ma, bg, mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    new Matrix(3, 3), ma, bg, mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    new Matrix(1, 3), bg, mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    new Matrix(3, 1), bg, mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, new Matrix(1, 1), mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, new Matrix(3, 3), mg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor28() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                ecefPosition, nedC, ba, ma, bg, mg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    ba, wrong, bg, mg, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    ba, ma, bg, wrong, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    new Matrix(1, 1), ma, bg, mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    new Matrix(3, 3), ma, bg, mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    new Matrix(1, 3), bg, mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    new Matrix(3, 1), bg, mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, new Matrix(1, 1), mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, new Matrix(3, 3), mg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor29() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                ecefPosition, nedC, ba, ma, bg, mg, gg);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    ba, wrong, bg, mg, gg);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    ba, ma, bg, wrong, gg);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    new Matrix(1, 1), ma, bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    new Matrix(3, 3), ma, bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    new Matrix(1, 3), bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    new Matrix(3, 1), bg, mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, new Matrix(1, 1), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, new Matrix(3, 3), mg, gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, new Matrix(1, 3), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, new Matrix(3, 1), gg);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, mg, new Matrix(1, 3));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, mg, new Matrix(3, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor30() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                ecefPosition, nedC, ba, ma, bg, mg, gg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    ba, wrong, bg, mg, gg, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    ba, ma, bg, wrong, gg, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    new Matrix(1, 1), ma, bg, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    new Matrix(3, 3), ma, bg, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    new Matrix(1, 3), bg, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    new Matrix(3, 1), bg, mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, new Matrix(1, 1), mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, new Matrix(3, 3), mg, gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, new Matrix(1, 3), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, new Matrix(3, 1), gg, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, mg, new Matrix(1, 3), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, mg, new Matrix(3, 1), this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor31() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();

        final double timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                nedPosition, nedC, baTriad, ma, bgTriad, mg, timeInterval);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    baTriad, wrong, bgTriad, mg, timeInterval);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    baTriad, ma, bgTriad, wrong, timeInterval);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    new Matrix(1, 3), bgTriad, mg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    new Matrix(3, 1), bgTriad, mg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(1, 3), timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(3, 1), timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor32() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();

        final double timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                nedPosition, nedC, baTriad, ma, bgTriad, mg, timeInterval,
                this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    baTriad, wrong, bgTriad, mg, timeInterval, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    baTriad, ma, bgTriad, wrong, timeInterval, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    new Matrix(1, 3), bgTriad, mg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    new Matrix(3, 1), bgTriad, mg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(1, 3), timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(3, 1), timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor33() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                nedPosition, nedC, baTriad, ma, bgTriad, mg, gg,
                timeInterval);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    baTriad, wrong, bgTriad, mg, gg, timeInterval);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    baTriad, ma, bgTriad, wrong, gg, timeInterval);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    new Matrix(1, 3), bgTriad, mg, gg,
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    new Matrix(3, 1), bgTriad, mg, gg,
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(1, 3), gg,
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(3, 1), gg,
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, mg, new Matrix(1, 3),
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, mg, new Matrix(3, 1),
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor34() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                nedPosition, nedC, baTriad, ma, bgTriad, mg, gg,
                timeInterval, this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    baTriad, wrong, bgTriad, mg, gg, timeInterval, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    baTriad, ma, bgTriad, wrong, gg, timeInterval, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    new Matrix(1, 3), bgTriad, mg, gg,
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    new Matrix(3, 1), bgTriad, mg, gg,
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(1, 3), gg,
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(3, 1), gg,
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, mg, new Matrix(1, 3),
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad,
                    ma, bgTriad, mg, new Matrix(3, 1),
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor35() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();

        final double timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                nedPosition, nedC, ba, ma, bg, mg, timeInterval);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    ba, wrong, bg, mg, timeInterval);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    ba, ma, bg, wrong, timeInterval);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    new Matrix(1, 1), ma, bg, mg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    new Matrix(3, 3), ma, bg, mg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    new Matrix(1, 3), bg, mg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    new Matrix(3, 1), bg, mg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, new Matrix(1, 1), mg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, new Matrix(3, 3), mg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, new Matrix(1, 3), timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, new Matrix(3, 1), timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor36() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();

        final double timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                nedPosition, nedC, ba, ma, bg, mg, timeInterval, this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    ba, wrong, bg, mg, timeInterval, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    ba, ma, bg, wrong, timeInterval, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    new Matrix(1, 1), ma, bg, mg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    new Matrix(3, 3), ma, bg, mg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    new Matrix(1, 3), bg, mg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    new Matrix(3, 1), bg, mg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, new Matrix(1, 1), mg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, new Matrix(3, 3), mg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, new Matrix(1, 3), timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, new Matrix(3, 1), timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor37() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                nedPosition, nedC, ba, ma, bg, mg, gg, timeInterval);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    ba, wrong, bg, mg, gg, timeInterval);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    ba, ma, bg, wrong, gg, timeInterval);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    new Matrix(1, 1), ma, bg, mg, gg,
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    new Matrix(3, 3), ma, bg, mg, gg,
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    new Matrix(1, 3), bg, mg, gg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    new Matrix(3, 1), bg, mg, gg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, new Matrix(1, 1), mg, gg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, new Matrix(3, 3), mg, gg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, new Matrix(1, 3), gg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, new Matrix(3, 1), gg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, mg, new Matrix(1, 3), timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, mg, new Matrix(3, 1), timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor38() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                nedPosition, nedC, ba, ma, bg, mg, gg, timeInterval,
                this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    ba, wrong, bg, mg, gg, timeInterval, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    ba, ma, bg, wrong, gg, timeInterval, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    new Matrix(1, 1), ma, bg, mg, gg,
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC,
                    new Matrix(3, 3), ma, bg, mg, gg,
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    new Matrix(1, 3), bg, mg, gg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    new Matrix(3, 1), bg, mg, gg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, new Matrix(1, 1), mg, gg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, new Matrix(3, 3), mg, gg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, new Matrix(1, 3), gg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, new Matrix(3, 1), gg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, mg, new Matrix(1, 3), timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(nedPosition, nedC, ba,
                    ma, bg, mg, new Matrix(3, 1), timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor39() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();

        final double timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                ecefPosition, nedC, baTriad, ma, bgTriad, mg, timeInterval);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    baTriad, wrong, bgTriad, mg, timeInterval);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    baTriad, ma, bgTriad, wrong, timeInterval);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    new Matrix(1, 3), bgTriad, mg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    new Matrix(3, 1), bgTriad, mg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(1, 3), timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(3, 1), timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor40() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();

        final double timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                ecefPosition, nedC, baTriad, ma, bgTriad, mg, timeInterval,
                this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    baTriad, wrong, bgTriad, mg, timeInterval, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    baTriad, ma, bgTriad, wrong, timeInterval, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    new Matrix(1, 3), bgTriad, mg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    new Matrix(3, 1), bgTriad, mg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(1, 3), timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(3, 1), timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor41() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                ecefPosition, nedC, baTriad, ma, bgTriad, mg, gg,
                timeInterval);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    baTriad, wrong, bgTriad, mg, gg, timeInterval);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    baTriad, ma, bgTriad, wrong, gg, timeInterval);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    new Matrix(1, 3), bgTriad, mg, gg,
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    new Matrix(3, 1), bgTriad, mg, gg,
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(1, 3), gg,
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(3, 1), gg,
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, mg, new Matrix(1, 3),
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, mg, new Matrix(3, 1),
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor42() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                ecefPosition, nedC, baTriad, ma, bgTriad, mg, gg, timeInterval,
                this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    baTriad, wrong, bgTriad, mg, gg, timeInterval, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    baTriad, ma, bgTriad, wrong, gg, timeInterval, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    new Matrix(1, 3), bgTriad, mg, gg,
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    new Matrix(3, 1), bgTriad, mg, gg,
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(1, 3), gg,
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, new Matrix(3, 1), gg,
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, mg, new Matrix(1, 3),
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                    ma, bgTriad, mg, new Matrix(3, 1),
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor43() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();

        final double timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                ecefPosition, nedC, ba, ma, bg, mg, timeInterval);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    ba, wrong, bg, mg, timeInterval);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    ba, ma, bg, wrong, timeInterval);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    new Matrix(1, 1), ma, bg, mg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    new Matrix(3, 3), ma, bg, mg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    new Matrix(1, 3), bg, mg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    new Matrix(3, 1), bg, mg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, new Matrix(1, 1), mg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, new Matrix(3, 3), mg, timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, new Matrix(1, 3), timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, new Matrix(3, 1), timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor44() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();

        final double timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                ecefPosition, nedC, ba, ma, bg, mg, timeInterval, this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    ba, wrong, bg, mg, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    ba, ma, bg, wrong, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    new Matrix(1, 1), ma, bg, mg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    new Matrix(3, 3), ma, bg, mg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    new Matrix(1, 3), bg, mg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    new Matrix(3, 1), bg, mg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, new Matrix(1, 1), mg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, new Matrix(3, 3), mg, timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, new Matrix(1, 3), timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, new Matrix(3, 1), timeInterval,
                    this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor45() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                ecefPosition, nedC, ba, ma, bg, mg, gg, timeInterval);

        // check default values
        assertNull(estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    ba, wrong, bg, mg, gg, timeInterval);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    ba, ma, bg, wrong, gg, timeInterval);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    new Matrix(1, 1), ma, bg, mg, gg,
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    new Matrix(3, 3), ma, bg, mg, gg,
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    new Matrix(1, 3), bg, mg, gg,
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    new Matrix(3, 1), bg, mg, gg,
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, new Matrix(1, 1), mg, gg,
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, new Matrix(3, 3), mg, gg,
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, new Matrix(1, 3), gg,
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, new Matrix(3, 1), gg,
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, mg, new Matrix(1, 3),
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, mg, new Matrix(3, 1),
                    timeInterval);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testConstructor46() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException {
        final NEDPosition nedPosition = createPosition();
        final CoordinateTransformation nedC = createOrientation();
        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final Matrix ba = generateBa();
        final AccelerationTriad baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final Matrix ma = generateMaGeneral();
        final Matrix bg = generateBg();
        final AngularSpeedTriad bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        RandomWalkEstimator estimator = new RandomWalkEstimator(
                ecefPosition, nedC, ba, ma, bg, mg, gg, timeInterval,
                this);

        // check default values
        assertSame(this, estimator.getListener());

        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final Matrix ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final double[] ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final AccelerationTriad triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(),
                estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(),
                estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(),
                estimator.getAccelerationBiasZ(), 0.0);

        final Acceleration bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final Acceleration bax2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final Acceleration bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final Acceleration bay2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final Acceleration baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final Acceleration baz2 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final Matrix ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        double sx = ma.getElementAt(0, 0);
        double sy = ma.getElementAt(1, 1);
        double sz = ma.getElementAt(2, 2);
        double mxy = ma.getElementAt(0, 1);
        double mxz = ma.getElementAt(0, 2);
        double myx = ma.getElementAt(1, 0);
        double myz = ma.getElementAt(1, 2);
        double mzx = ma.getElementAt(2, 0);
        double mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final Matrix bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(),
                estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(),
                estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(),
                estimator.getAngularSpeedBiasZ(), 0.0);

        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final AngularSpeed bgx2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final AngularSpeed bgy2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final AngularSpeed bgz2 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final Matrix mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final Matrix gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final Time t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final NEDFrame nedFrame1 = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(),
                LARGE_ABSOLUTE_ERROR));
        final ECEFPosition ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), ABSOLUTE_ERROR));
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final NEDPosition nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertTrue(estimator.isFixKinematicsEnabled());
        assertFalse(estimator.isRunning());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertEquals(0.0, estimator.getPositionStandardDeviation(),
                0.0);
        final Distance positionStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(0.0, positionStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionStd1.getUnit());
        final Distance positionStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(positionStd2);
        assertEquals(positionStd1, positionStd2);

        assertEquals(0.0, estimator.getVelocityStandardDeviation(),
                0.0);
        final Speed speedStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(0.0, speedStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, speedStd1.getUnit());
        final Speed speedStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(speedStd2);
        assertEquals(speedStd1, speedStd2);

        assertEquals(0.0, estimator.getAttitudeStandardDeviation(), 0.0);
        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(0.0, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());
        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        final BodyKinematics kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final BodyKinematics kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        estimator = null;
        final Matrix wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    ba, wrong, bg, mg, gg, timeInterval, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    ba, ma, bg, wrong, gg, timeInterval, this);
            fail("AlgebraException expected but not thrown");
        } catch (final AlgebraException ignore) {
        }

        // Force IllegalArgumentException
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    new Matrix(1, 1), ma, bg, mg, gg,
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC,
                    new Matrix(3, 3), ma, bg, mg, gg,
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    new Matrix(1, 3), bg, mg, gg,
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    new Matrix(3, 1), bg, mg, gg,
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, new Matrix(1, 1), mg, gg,
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, new Matrix(3, 3), mg, gg,
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, new Matrix(1, 3), gg,
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, new Matrix(3, 1), gg,
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, mg, new Matrix(1, 3),
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator = new RandomWalkEstimator(ecefPosition, nedC, ba,
                    ma, bg, mg, new Matrix(3, 1),
                    timeInterval, this);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        assertNull(estimator);
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    public void testGetSetAccelerationBias1()
            throws WrongSizeException, LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        final Matrix ba1 = estimator.getAccelerationBias();
        assertEquals(ba1, new Matrix(3, 1));

        // set new value
        final Matrix ba2 = generateBa();
        estimator.setAccelerationBias(ba2);

        // check
        final Matrix ba3 = estimator.getAccelerationBias();
        final Matrix ba4 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba4);
        assertEquals(ba2, ba3);
        assertEquals(ba2, ba4);

        // Force IllegalArgumentException
        try {
            estimator.setAccelerationBias(new Matrix(1, 1));
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAccelerationBiasArray() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        final double[] ba1 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba1, new double[3], 0.0);

        // set new value
        final double[] ba2 = generateBa().getBuffer();
        estimator.setAccelerationBias(ba2);

        // check
        final double[] ba3 = estimator.getAccelerationBiasArray();
        final double[] ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba2, ba3, 0.0);
        assertArrayEquals(ba2, ba4, 0.0);

        // Force IllegalArgumentException
        try {
            estimator.getAccelerationBiasArray(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
        try {
            estimator.setAccelerationBias(new double[1]);
            fail("IllegalArgumentException expected but not thrown");
        } catch (final IllegalArgumentException ignore) {
        }
    }

    @Test
    public void testGetSetAccelerationBiasAsTriad() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        final AccelerationTriad triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        final AccelerationTriad triad2 = new AccelerationTriad(
                AccelerationUnit.METERS_PER_SQUARED_SECOND, bx, by, bz);
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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        estimator.setAccelerationBiasX(bx);

        // check
        assertEquals(bx, estimator.getAccelerationBiasX(), 0.0);
    }

    @Test
    public void testGetSetAccelerationBiasY() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double by = ba.getElementAtIndex(1);
        estimator.setAccelerationBiasY(by);

        // check
        assertEquals(by, estimator.getAccelerationBiasY(), 0.0);
    }

    @Test
    public void testGetSetAccelerationBiasZ() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double bz = ba.getElementAtIndex(2);
        estimator.setAccelerationBiasZ(bz);

        // check
        assertEquals(bz, estimator.getAccelerationBiasZ(), 0.0);
    }

    @Test
    public void testSetAccelerationBias() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        // set new value
        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final double by = ba.getElementAtIndex(1);
        final double bz = ba.getElementAtIndex(2);
        estimator.setAccelerationBias(bx, by, bz);

        // check
        assertEquals(bx, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(by, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(bz, estimator.getAccelerationBiasZ(), 0.0);
    }

    @Test
    public void testGetSetAccelerationBiasXAsAcceleration() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        final Acceleration bx1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bx1.getUnit());

        // set new value
        final Matrix ba = generateBa();
        final double bx = ba.getElementAtIndex(0);
        final Acceleration bx2 = new Acceleration(
                bx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        estimator.setAccelerationBiasX(bx2);

        // check
        final Acceleration bx3 = estimator.getAccelerationBiasXAsAcceleration();
        final Acceleration bx4 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bx4);
        assertEquals(bx2, bx3);
        assertEquals(bx2, bx4);
    }

    @Test
    public void testGetSetAccelerationBiasYAsAcceleration() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        final Acceleration by1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, by1.getUnit());

        // set new value
        final Matrix ba = generateBa();
        final double by = ba.getElementAtIndex(1);
        final Acceleration by2 = new Acceleration(
                by, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        estimator.setAccelerationBiasY(by2);

        // check
        final Acceleration by3 = estimator.getAccelerationBiasYAsAcceleration();
        final Acceleration by4 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(by4);
        assertEquals(by2, by3);
        assertEquals(by2, by4);
    }

    @Test
    public void testGetSetAccelerationBiasZAsAcceleration() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        final Acceleration bz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bz1.getUnit());

        // set new value
        final Matrix ba = generateBa();
        final double bz = ba.getElementAtIndex(2);
        final Acceleration bz2 = new Acceleration(
                bz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        estimator.setAccelerationBiasZ(bz2);

        // check
        final Acceleration bz3 = estimator.getAccelerationBiasZAsAcceleration();
        final Acceleration bz4 = new Acceleration(
                1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(bz4);
        assertEquals(bz2, bz3);
        assertEquals(bz2, bz4);
    }

    @Test
    public void testSetAccelerationBias2() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        // set new value
        final Matrix ba = generateBa();
        final Acceleration bx = new Acceleration(ba.getElementAtIndex(0),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration by = new Acceleration(ba.getElementAtIndex(1),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration bz = new Acceleration(ba.getElementAtIndex(2),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        estimator.setAccelerationBias(bx, by, bz);

        // check
        assertEquals(ba.getElementAtIndex(0), estimator.getAccelerationBiasX(),
                0.0);
        assertEquals(ba.getElementAtIndex(1), estimator.getAccelerationBiasY(),
                0.0);
        assertEquals(ba.getElementAtIndex(2), estimator.getAccelerationBiasZ(),
                0.0);
    }

    @Test
    public void testGetSetAccelerationCrossCouplingErrors()
            throws AlgebraException, LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        final Matrix ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), ma1);

        // set new values
        final Matrix ma2 = generateMaGeneral();
        estimator.setAccelerationCrossCouplingErrors(ma2);

        // check
        final Matrix ma3 = estimator.getAccelerationCrossCouplingErrors();
        final Matrix ma4 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma4);
        assertEquals(ma2, ma3);
        assertEquals(ma2, ma4);

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
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
    public void testSetAccelerationCrossCouplingErrors()
            throws AlgebraException, LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
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
    public void testSetAccelerationScalingFactorsAndCrossCouplingErrors()
            throws AlgebraException, LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
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

        assertEquals(ma, estimator.getAccelerationCrossCouplingErrors());
    }

    @Test
    public void testGetSetAngularSpeedBias1()
            throws WrongSizeException, LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        final Matrix bg1 = estimator.getAngularSpeedBias();
        assertEquals(new Matrix(3, 1), bg1);

        // set new value
        final Matrix bg2 = generateBg();
        estimator.setAngularSpeedBias(bg2);

        // check
        final Matrix bg3 = estimator.getAngularSpeedBias();
        final Matrix bg4 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg4);

        assertEquals(bg2, bg3);
        assertEquals(bg2, bg4);
    }

    @Test
    public void testGetSetAngularSpeedBiasArray() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        final double[] bg1 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(new double[3], bg1, 0.0);

        // set new values
        final double[] bg2 = generateBg().getBuffer();
        estimator.setAngularSpeedBias(bg2);

        // check
        final double[] bg3 = estimator.getAngularSpeedBiasArray();
        final double[] bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);

        assertArrayEquals(bg2, bg3, 0.0);
        assertArrayEquals(bg2, bg4, 0.0);
    }

    @Test
    public void testGetSetAngularSpeedBiasAsTriad() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        final AngularSpeedTriad triad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());

        // set new values
        final Matrix bg = generateBg();
        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);
        final AngularSpeedTriad triad2 = new AngularSpeedTriad(
                AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);
        estimator.setAngularSpeedBias(triad2);

        final AngularSpeedTriad triad3 = estimator.getAngularSpeedBiasAsTriad();
        final AngularSpeedTriad triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);

        // check
        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    public void testGetSetAngularSpeedBiasX() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        final AngularSpeed bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());

        // set new value
        final Matrix bg = generateBg();
        final double bgx = bg.getElementAtIndex(0);
        final AngularSpeed bgx2 = new AngularSpeed(
                bgx, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.setAngularSpeedBiasX(bgx2);

        // check
        final AngularSpeed bgx3 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        final AngularSpeed bgx4 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx4);

        assertEquals(bgx2, bgx3);
        assertEquals(bgx2, bgx4);
    }

    @Test
    public void testGetSetAngularSpeedBiasYAsAngularSpeed()
            throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        final AngularSpeed bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());

        // set new value
        final Matrix bg = generateBg();
        final double bgy = bg.getElementAtIndex(1);
        final AngularSpeed bgy2 = new AngularSpeed(
                bgy, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.setAngularSpeedBiasY(bgy2);

        // check
        final AngularSpeed bgy3 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        final AngularSpeed bgy4 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy4);

        assertEquals(bgy2, bgy3);
        assertEquals(bgy2, bgy4);
    }

    @Test
    public void testGetSetAngularSpeedBiasZAsAngularSpeed()
            throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        final AngularSpeed bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());

        // set new value
        final Matrix bg = generateBg();
        final double bgz = bg.getElementAtIndex(2);
        final AngularSpeed bgz2 = new AngularSpeed(
                bgz, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.setAngularSpeedBiasZ(bgz2);

        // check
        final AngularSpeed bgz3 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        final AngularSpeed bgz4 = new AngularSpeed(
                1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz4);

        assertEquals(bgz2, bgz3);
        assertEquals(bgz2, bgz4);
    }

    @Test
    public void testGetSetAngularSpeedBias2()
            throws WrongSizeException, LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);

        // set new value
        final Matrix bg = generateBg();
        final double bgx = bg.getElementAtIndex(0);
        final double bgy = bg.getElementAtIndex(1);
        final double bgz = bg.getElementAtIndex(2);

        final AngularSpeed bgx1 = new AngularSpeed(
                bgx, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgy1 = new AngularSpeed(
                bgy, AngularSpeedUnit.RADIANS_PER_SECOND);
        final AngularSpeed bgz1 = new AngularSpeed(
                bgz, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.setAngularSpeedBias(bgx1, bgy1, bgz1);

        // check
        final AngularSpeed bgx2 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        final AngularSpeed bgy2 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        final AngularSpeed bgz2 = estimator.getAngularSpeedBiasZAsAngularSpeed();

        assertEquals(bgx1, bgx2);
        assertEquals(bgy1, bgy2);
        assertEquals(bgz1, bgz2);
    }

    @Test
    public void testGetSetAngularSpeedCrossCouplingErrors()
            throws AlgebraException, LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        final Matrix mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), mg1);

        // set new values
        final Matrix mg2 = generateMg();
        estimator.setAngularSpeedCrossCouplingErrors(mg2);

        // check
        final Matrix mg3 = estimator.getAngularSpeedCrossCouplingErrors();
        final Matrix mg4 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg4);
        assertEquals(mg2, mg3);
        assertEquals(mg2, mg4);

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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

        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
    }

    @Test
    public void testSetAngularSpeedCrossCouplingErrors()
            throws AlgebraException, LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

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
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        final Matrix gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);

        final Matrix gg2 = generateGg();
        estimator.setAngularSpeedGDependantCrossBias(gg2);

        // check
        final Matrix gg3 = estimator.getAngularSpeedGDependantCrossBias();
        final Matrix gg4 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg4);
        assertEquals(gg2, gg3);
        assertEquals(gg2, gg4);

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
    public void testGetSetTimeInterval() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        assertEquals(TIME_INTERVAL_SECONDS, estimator.getTimeInterval(),
                0.0);

        // set new value
        final double timeInterval = 2.0 * TIME_INTERVAL_SECONDS;
        estimator.setTimeInterval(timeInterval);

        // check
        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);
    }

    @Test
    public void testGetSetTimeIntervalAsTime() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        final Time t1 = estimator.getTimeIntervalAsTime();
        assertEquals(TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        // set new value
        final Time t2 = new Time(2.0 * TIME_INTERVAL_SECONDS,
                TimeUnit.SECOND);
        estimator.setTimeInterval(t2);

        // check
        final Time t3 = estimator.getTimeIntervalAsTime();
        final Time t4 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t4);
        assertEquals(t2, t3);
        assertEquals(t2, t4);
    }

    @Test
    public void testGetSetEcefPosition() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        final ECEFPosition ecefPosition1 =
                new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0, 0.0);
        assertEquals(estimator.getEcefPosition(), ecefPosition1);

        // set new value
        final ECEFPosition ecefPosition2 =
                new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                        Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                        Constants.EARTH_EQUATORIAL_RADIUS_WGS84);
        estimator.setEcefPosition(ecefPosition2);

        // check
        final ECEFPosition ecefPosition3 = estimator.getEcefPosition();
        final ECEFPosition ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefPosition2, ecefPosition4);
    }

    @Test
    public void testSetEcefPosition1() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        final ECEFPosition ecefPosition1 =
                new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0, 0.0);
        assertEquals(estimator.getEcefPosition(), ecefPosition1);

        //  set new value
        estimator.setEcefPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84);

        // check
        final ECEFPosition ecefPosition2 =
                new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                        Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                        Constants.EARTH_EQUATORIAL_RADIUS_WGS84);

        final ECEFPosition ecefPosition3 = estimator.getEcefPosition();
        final ECEFPosition ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefPosition2, ecefPosition4);
    }

    @Test
    public void testSetEcefPosition2() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        final ECEFPosition ecefPosition1 =
                new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0, 0.0);
        assertEquals(estimator.getEcefPosition(), ecefPosition1);

        //  set new value
        final Distance distance = new Distance(
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84, DistanceUnit.METER);
        estimator.setEcefPosition(distance, distance, distance);

        // check
        final ECEFPosition ecefPosition2 =
                new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                        Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                        Constants.EARTH_EQUATORIAL_RADIUS_WGS84);

        final ECEFPosition ecefPosition3 = estimator.getEcefPosition();
        final ECEFPosition ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefPosition2, ecefPosition4);
    }

    @Test
    public void testSetEcefPosition3() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        final ECEFPosition ecefPosition1 =
                new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0, 0.0);
        assertEquals(estimator.getEcefPosition(), ecefPosition1);

        //  set new value
        final Point3D position = new InhomogeneousPoint3D(
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84);
        estimator.setEcefPosition(position);

        // check
        final ECEFPosition ecefPosition2 =
                new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                        Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                        Constants.EARTH_EQUATORIAL_RADIUS_WGS84);

        final ECEFPosition ecefPosition3 = estimator.getEcefPosition();
        final ECEFPosition ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefPosition2, ecefPosition4);
    }

    @Test
    public void testGetEcefFrame() {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        final NEDFrame nedFrame1 = new NEDFrame();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);

        assertEquals(estimator.getEcefFrame(), ecefFrame1);
        final ECEFFrame ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);
    }

    @Test
    public void testGetNedFrame() {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        final NEDFrame nedFrame1 = new NEDFrame();

        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        final NEDFrame nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
    }

    @Test
    public void testGetSetNedPosition() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        final NEDPosition nedPosition1 = estimator.getNedPosition();

        assertEquals(nedPosition1.getLatitude(), 0.0, ABSOLUTE_ERROR);
        assertEquals(nedPosition1.getLongitude(), 0.0, ABSOLUTE_ERROR);
        assertEquals(nedPosition1.getHeight(), 0.0, ABSOLUTE_ERROR);

        // set new value
        final NEDPosition nedPosition2 = createPosition();
        estimator.setNedPosition(nedPosition2);

        // check
        final NEDPosition nedPosition3 = estimator.getNedPosition();
        final NEDPosition nedPosition4 = new NEDPosition();
        estimator.getNedPosition(nedPosition4);

        assertTrue(nedPosition2.equals(nedPosition3, ABSOLUTE_ERROR));
        assertEquals(nedPosition3, nedPosition4);
    }

    @Test
    public void testSetNedPosition1() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(),
                ABSOLUTE_ERROR));

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        estimator.setNedPosition(latitude, longitude, height);

        // check
        final NEDPosition nedPosition2 = new NEDPosition(latitude, longitude, height);

        final NEDPosition nedPosition3 = estimator.getNedPosition();
        final NEDPosition nedPosition4 = new NEDPosition();
        estimator.getNedPosition(nedPosition4);

        assertTrue(nedPosition2.equals(nedPosition3, ABSOLUTE_ERROR));
        assertEquals(nedPosition3, nedPosition4);
    }

    @Test
    public void testSetNedPosition2() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(),
                ABSOLUTE_ERROR));

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Angle latitude = new Angle(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES),
                AngleUnit.DEGREES);
        final Angle longitude = new Angle(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final Distance heightDistance = new Distance(height, DistanceUnit.METER);

        estimator.setNedPosition(latitude, longitude, height);

        // check
        final NEDPosition nedPosition2 = new NEDPosition(latitude, longitude,
                heightDistance);

        final NEDPosition nedPosition3 = estimator.getNedPosition();
        final NEDPosition nedPosition4 = new NEDPosition();
        estimator.getNedPosition(nedPosition4);

        assertTrue(nedPosition2.equals(nedPosition3, ABSOLUTE_ERROR));
        assertEquals(nedPosition3, nedPosition4);
    }

    @Test
    public void testSetNedPosition3() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(),
                ABSOLUTE_ERROR));

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Angle latitude = new Angle(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES),
                AngleUnit.DEGREES);
        final Angle longitude = new Angle(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final Distance height = new Distance(
                randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT), DistanceUnit.METER);

        estimator.setNedPosition(latitude, longitude, height);

        // check
        final NEDPosition nedPosition2 = new NEDPosition(latitude, longitude, height);

        final NEDPosition nedPosition3 = estimator.getNedPosition();
        final NEDPosition nedPosition4 = new NEDPosition();
        estimator.getNedPosition(nedPosition4);

        assertTrue(nedPosition2.equals(nedPosition3, ABSOLUTE_ERROR));
        assertEquals(nedPosition3, nedPosition4);
    }

    @Test
    public void testGetSetEcefC()
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(new NEDFrame());
        final CoordinateTransformation ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(estimator.getEcefC(), ecefC1);

        final CoordinateTransformation ecefC2 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        estimator.setEcefC(ecefC2);

        // check
        final CoordinateTransformation ecefC3 = estimator.getEcefC();
        final CoordinateTransformation ecefC4 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC4);

        assertEquals(ecefC2, ecefC3);
        assertEquals(ecefC2, ecefC4);
    }

    @Test
    public void testGetSetNedC()
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        assertTrue(estimator.getNedC().equals(
                new CoordinateTransformation(
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME),
                ABSOLUTE_ERROR));

        // set new value
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final CoordinateTransformation nedC1 = new CoordinateTransformation(
                roll, pitch, yaw, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        estimator.setNedC(nedC1);

        final CoordinateTransformation nedC2 = estimator.getNedC();
        final CoordinateTransformation nedC3 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC3);

        assertTrue(nedC1.equals(nedC2, ABSOLUTE_ERROR));
        assertTrue(nedC1.equals(nedC3, ABSOLUTE_ERROR));
    }

    @Test
    public void testSetNedPositionAndNedOrientation1()
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(),
                ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(new CoordinateTransformation(
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME),
                ABSOLUTE_ERROR));

        // set new values
        final NEDPosition nedPosition1 = createPosition();
        final CoordinateTransformation nedC1 = createOrientation();

        estimator.setNedPositionAndNedOrientation(nedPosition1, nedC1);

        // check
        final NEDPosition nedPosition2 = estimator.getNedPosition();
        final CoordinateTransformation nedC2 = estimator.getNedC();

        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(nedC1.equals(nedC2, ABSOLUTE_ERROR));

        // Force InvalidSourceAndDestinationFrameTypeException
        try {
            estimator.setNedPositionAndNedOrientation(nedPosition1,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
        }
    }

    @Test
    public void testSetNedPositionAndNedOrientation2()
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(),
                ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(new CoordinateTransformation(
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME),
                ABSOLUTE_ERROR));

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final CoordinateTransformation nedC1 = createOrientation();

        estimator.setNedPositionAndNedOrientation(latitude, longitude, height, nedC1);

        // check
        final NEDPosition nedPosition1 = new NEDPosition(latitude, longitude, height);

        final NEDPosition nedPosition2 = estimator.getNedPosition();
        final CoordinateTransformation nedC2 = estimator.getNedC();

        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(nedC1.equals(nedC2, ABSOLUTE_ERROR));

        // Force InvalidSourceAndDestinationFrameTypeException
        try {
            estimator.setNedPositionAndNedOrientation(latitude, longitude, height,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
        }
    }

    @Test
    public void testSetNedPositionAndNedOrientation3()
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(),
                ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(new CoordinateTransformation(
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME),
                ABSOLUTE_ERROR));

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Angle latitude = new Angle(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES),
                AngleUnit.DEGREES);
        final Angle longitude = new Angle(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final Distance heightDistance = new Distance(height, DistanceUnit.METER);

        final CoordinateTransformation nedC1 = createOrientation();

        estimator.setNedPositionAndNedOrientation(latitude, longitude, height, nedC1);

        // check
        final NEDPosition nedPosition1 = new NEDPosition(latitude, longitude,
                heightDistance);

        final NEDPosition nedPosition2 = estimator.getNedPosition();
        final CoordinateTransformation nedC2 = estimator.getNedC();

        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(nedC1.equals(nedC2, ABSOLUTE_ERROR));

        // Force InvalidSourceAndDestinationFrameTypeException
        try {
            estimator.setNedPositionAndNedOrientation(latitude, longitude,
                    height,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
        }
    }

    @Test
    public void testSetNedPositionAndNedOrientation4()
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(),
                ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(new CoordinateTransformation(
                        FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME),
                ABSOLUTE_ERROR));

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Angle latitude = new Angle(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES),
                AngleUnit.DEGREES);
        final Angle longitude = new Angle(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final Distance heightDistance = new Distance(height, DistanceUnit.METER);

        final CoordinateTransformation nedC1 = createOrientation();

        estimator.setNedPositionAndNedOrientation(latitude, longitude, heightDistance,
                nedC1);

        // check
        final NEDPosition nedPosition1 = new NEDPosition(latitude, longitude,
                heightDistance);

        final NEDPosition nedPosition2 = estimator.getNedPosition();
        final CoordinateTransformation nedC2 = estimator.getNedC();

        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(nedC1.equals(nedC2, ABSOLUTE_ERROR));

        // Force InvalidSourceAndDestinationFrameTypeException
        try {
            estimator.setNedPositionAndNedOrientation(latitude, longitude,
                    heightDistance,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
        }
    }

    @Test
    public void testSetEcefPositionAndEcefOrientation1()
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(new NEDFrame());
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1
                .getCoordinateTransformation();

        assertEquals(estimator.getEcefPosition(), ecefPosition1);
        assertEquals(estimator.getEcefC(), ecefC1);

        // set new values
        final NEDPosition nedPosition2 = createPosition();
        final CoordinateTransformation nedC2 = createOrientation();

        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);

        final ECEFPosition ecefPosition2 = ecefFrame2.getECEFPosition();
        final CoordinateTransformation ecefC2 = ecefFrame2
                .getCoordinateTransformation();

        estimator.setEcefPositionAndEcefOrientation(ecefPosition2, ecefC2);

        // check
        final ECEFPosition ecefPosition3 = estimator.getEcefPosition();
        final ECEFPosition ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        final CoordinateTransformation ecefC3 = estimator.getEcefC();
        final CoordinateTransformation ecefC4 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefC2, ecefC3);

        assertEquals(ecefPosition2, ecefPosition4);
        assertEquals(ecefC2, ecefC4);

        // Force InvalidSourceAndDestinationFrameTypeException
        try {
            estimator.setEcefPositionAndEcefOrientation(ecefPosition1,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
        }
    }

    @Test
    public void testSetEcefPositionAndEcefOrientation2()
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(new NEDFrame());
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1
                .getCoordinateTransformation();

        assertEquals(estimator.getEcefPosition(), ecefPosition1);
        assertEquals(estimator.getEcefC(), ecefC1);

        // set new values
        final NEDPosition nedPosition2 = createPosition();
        final CoordinateTransformation nedC2 = createOrientation();

        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);

        final ECEFPosition ecefPosition2 = ecefFrame2.getECEFPosition();
        final CoordinateTransformation ecefC2 = ecefFrame2
                .getCoordinateTransformation();

        final double x = ecefPosition2.getX();
        final double y = ecefPosition2.getY();
        final double z = ecefPosition2.getZ();
        estimator.setEcefPositionAndEcefOrientation(x, y, z, ecefC2);

        // check
        final ECEFPosition ecefPosition3 = estimator.getEcefPosition();
        final ECEFPosition ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        final CoordinateTransformation ecefC3 = estimator.getEcefC();
        final CoordinateTransformation ecefC4 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefC2, ecefC3);

        assertEquals(ecefPosition2, ecefPosition4);
        assertEquals(ecefC2, ecefC4);

        // Force InvalidSourceAndDestinationFrameTypeException
        try {
            estimator.setEcefPositionAndEcefOrientation(x, y, z,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
        }
    }

    @Test
    public void testSetEcefPositionAndEcefOrientation3()
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(new NEDFrame());
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1
                .getCoordinateTransformation();

        assertEquals(estimator.getEcefPosition(), ecefPosition1);
        assertEquals(estimator.getEcefC(), ecefC1);

        // set new values
        final NEDPosition nedPosition2 = createPosition();
        final CoordinateTransformation nedC2 = createOrientation();

        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);

        final ECEFPosition ecefPosition2 = ecefFrame2.getECEFPosition();
        final CoordinateTransformation ecefC2 = ecefFrame2
                .getCoordinateTransformation();

        final Distance x = new Distance(ecefPosition2.getX(), DistanceUnit.METER);
        final Distance y = new Distance(ecefPosition2.getY(), DistanceUnit.METER);
        final Distance z = new Distance(ecefPosition2.getZ(), DistanceUnit.METER);
        estimator.setEcefPositionAndEcefOrientation(x, y, z, ecefC2);

        // check
        final ECEFPosition ecefPosition3 = estimator.getEcefPosition();
        final ECEFPosition ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        final CoordinateTransformation ecefC3 = estimator.getEcefC();
        final CoordinateTransformation ecefC4 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefC2, ecefC3);

        assertEquals(ecefPosition2, ecefPosition4);
        assertEquals(ecefC2, ecefC4);

        // Force InvalidSourceAndDestinationFrameTypeException
        try {
            estimator.setEcefPositionAndEcefOrientation(x, y, z,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
        }
    }

    @Test
    public void testSetEcefPositionAndEcefOrientation4()
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(new NEDFrame());
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1
                .getCoordinateTransformation();

        assertEquals(estimator.getEcefPosition(), ecefPosition1);
        assertEquals(estimator.getEcefC(), ecefC1);

        // set new values
        final NEDPosition nedPosition2 = createPosition();
        final CoordinateTransformation nedC2 = createOrientation();

        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);

        final ECEFPosition ecefPosition2 = ecefFrame2.getECEFPosition();
        final CoordinateTransformation ecefC2 = ecefFrame2
                .getCoordinateTransformation();

        final double x = ecefPosition2.getX();
        final double y = ecefPosition2.getY();
        final double z = ecefPosition2.getZ();
        final Point3D point = new InhomogeneousPoint3D(x, y, z);
        estimator.setEcefPositionAndEcefOrientation(point, ecefC2);

        // check
        final ECEFPosition ecefPosition3 = estimator.getEcefPosition();
        final ECEFPosition ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        final CoordinateTransformation ecefC3 = estimator.getEcefC();
        final CoordinateTransformation ecefC4 = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefC2, ecefC3);

        assertEquals(ecefPosition2, ecefPosition4);
        assertEquals(ecefC2, ecefC4);

        // Force InvalidSourceAndDestinationFrameTypeException
        try {
            estimator.setEcefPositionAndEcefOrientation(point,
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
        }
    }

    @Test
    public void testSetNedPositionAndEcefOrientation1()
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        final NEDFrame nedFrame1 = new NEDFrame();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);

        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1
                .getCoordinateTransformation();
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1
                .getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(estimator.getEcefPosition(), ecefPosition1);
        assertEquals(estimator.getEcefC(), ecefC1);

        // set new values
        final NEDPosition nedPosition2 = createPosition();
        final CoordinateTransformation nedC2 = createOrientation();

        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);

        final ECEFPosition ecefPosition2 = ecefFrame2.getECEFPosition();
        final CoordinateTransformation ecefC2 = ecefFrame2
                .getCoordinateTransformation();

        estimator.setNedPositionAndEcefOrientation(nedPosition2, ecefC2);

        // check
        assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
        assertEquals(estimator.getEcefPosition(), ecefPosition2);
        assertEquals(estimator.getEcefC(), ecefC2);

        // Force InvalidSourceAndDestinationFrameTypeException
        try {
            estimator.setNedPositionAndEcefOrientation(nedPosition1,
                    new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
        }
    }

    @Test
    public void testSetNedPositionAndEcefOrientation2()
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        final NEDFrame nedFrame1 = new NEDFrame();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);

        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1
                .getCoordinateTransformation();
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1
                .getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(estimator.getEcefPosition(), ecefPosition1);
        assertEquals(estimator.getEcefC(), ecefC1);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition2 = new NEDPosition(latitude, longitude, height);

        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                roll, pitch, yaw, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);

        final ECEFPosition ecefPosition2 = ecefFrame2.getECEFPosition();
        final CoordinateTransformation ecefC2 = ecefFrame2
                .getCoordinateTransformation();

        estimator.setNedPositionAndEcefOrientation(latitude, longitude, height,
                ecefC2);

        // check
        assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
        assertEquals(estimator.getEcefPosition(), ecefPosition2);
        assertEquals(estimator.getEcefC(), ecefC2);

        // Force InvalidSourceAndDestinationFrameTypeException
        try {
            estimator.setNedPositionAndEcefOrientation(latitude, longitude, height,
                    new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
        }
    }

    @Test
    public void testSetNedPositionAndEcefOrientation3()
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        final NEDFrame nedFrame1 = new NEDFrame();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);

        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1
                .getCoordinateTransformation();
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1
                .getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(estimator.getEcefPosition(), ecefPosition1);
        assertEquals(estimator.getEcefC(), ecefC1);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Angle latitude = new Angle(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES),
                AngleUnit.DEGREES);
        final Angle longitude = new Angle(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final Distance heightDistance = new Distance(height, DistanceUnit.METER);
        final NEDPosition nedPosition2 = new NEDPosition(latitude, longitude,
                heightDistance);

        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);

        final ECEFPosition ecefPosition2 = ecefFrame2.getECEFPosition();
        final CoordinateTransformation ecefC2 = ecefFrame2
                .getCoordinateTransformation();

        estimator.setNedPositionAndEcefOrientation(latitude, longitude, height,
                ecefC2);

        // check
        assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
        assertEquals(estimator.getEcefPosition(), ecefPosition2);
        assertEquals(estimator.getEcefC(), ecefC2);

        // Force InvalidSourceAndDestinationFrameTypeException
        try {
            estimator.setNedPositionAndEcefOrientation(latitude, longitude, height,
                    new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
        }
    }

    @Test
    public void testSetNedPositionAndEcefOrientation4()
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        final NEDFrame nedFrame1 = new NEDFrame();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);

        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1
                .getCoordinateTransformation();
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1
                .getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(estimator.getEcefPosition(), ecefPosition1);
        assertEquals(estimator.getEcefC(), ecefC1);

        // set new values
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final Angle latitude = new Angle(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES),
                AngleUnit.DEGREES);
        final Angle longitude = new Angle(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final Distance height = new Distance(
                randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT), DistanceUnit.METER);
        final NEDPosition nedPosition2 = new NEDPosition(latitude, longitude,
                height);

        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final CoordinateTransformation nedC2 = new CoordinateTransformation(
                roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);

        final ECEFPosition ecefPosition2 = ecefFrame2.getECEFPosition();
        final CoordinateTransformation ecefC2 = ecefFrame2
                .getCoordinateTransformation();

        estimator.setNedPositionAndEcefOrientation(latitude, longitude, height,
                ecefC2);

        // check
        assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
        assertEquals(estimator.getEcefPosition(), ecefPosition2);
        assertEquals(estimator.getEcefC(), ecefC2);

        // Force InvalidSourceAndDestinationFrameTypeException
        try {
            estimator.setNedPositionAndEcefOrientation(latitude, longitude, height,
                    new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
        }
    }

    @Test
    public void testSetEcefPositionAndNedOrientation1()
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        final NEDFrame nedFrame1 = new NEDFrame();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);

        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1
                .getCoordinateTransformation();
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1
                .getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(estimator.getEcefPosition(), ecefPosition1);
        assertEquals(estimator.getEcefC(), ecefC1);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            // set new values
            final NEDPosition nedPosition2 = createPosition();
            final CoordinateTransformation nedC2 = createOrientation();

            final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, nedC2);
            final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame2);

            final ECEFPosition ecefPosition2 = ecefFrame2.getECEFPosition();
            final CoordinateTransformation ecefC2 = ecefFrame2
                    .getCoordinateTransformation();

            estimator.setEcefPositionAndNedOrientation(ecefPosition2, nedC2);

            // check
            assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
            if (!estimator.getEcefPosition().equals(ecefPosition2, 5.0 * ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefPosition().equals(ecefPosition2,
                    5.0 * ABSOLUTE_ERROR));
            assertTrue(estimator.getEcefC().equals(ecefC2, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // Force InvalidSourceAndDestinationFrameTypeException
        try {
            estimator.setEcefPositionAndNedOrientation(ecefPosition1,
                    new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
        }
    }

    @Test
    public void testSetEcefPositionAndNedOrientation2()
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final RandomWalkEstimator estimator = new RandomWalkEstimator();

            // check default values
            final NEDFrame nedFrame1 = new NEDFrame();
            final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame1);

            final NEDPosition nedPosition1 = nedFrame1.getPosition();
            final CoordinateTransformation nedC1 = nedFrame1
                    .getCoordinateTransformation();
            final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
            final CoordinateTransformation ecefC1 = ecefFrame1
                    .getCoordinateTransformation();

            assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
            assertEquals(estimator.getEcefPosition(), ecefPosition1);
            assertEquals(estimator.getEcefC(), ecefC1);

            // set new values
            final NEDPosition nedPosition2 = createPosition();
            final CoordinateTransformation nedC2 = createOrientation();

            final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, nedC2);
            final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame2);

            final ECEFPosition ecefPosition2 = ecefFrame2.getECEFPosition();
            final CoordinateTransformation ecefC2 = ecefFrame2
                    .getCoordinateTransformation();

            final double x = ecefPosition2.getX();
            final double y = ecefPosition2.getY();
            final double z = ecefPosition2.getZ();

            estimator.setEcefPositionAndNedOrientation(x, y, z, nedC2);

            // check
            if (!estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
            if (!estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
            if (!estimator.getEcefPosition().equals(ecefPosition2,
                    LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefPosition().equals(ecefPosition2,
                    LARGE_ABSOLUTE_ERROR));
            if (!estimator.getEcefC().equals(ecefC2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefC().equals(ecefC2, ABSOLUTE_ERROR));

            // Force InvalidSourceAndDestinationFrameTypeException
            try {
                estimator.setEcefPositionAndNedOrientation(x, y, z,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.BODY_FRAME));
                fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
            } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSetEcefPositionAndNedOrientation3()
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        final NEDFrame nedFrame1 = new NEDFrame();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);

        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1
                .getCoordinateTransformation();
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1
                .getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(estimator.getEcefPosition(), ecefPosition1);
        assertEquals(estimator.getEcefC(), ecefC1);

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            // set new values
            final NEDPosition nedPosition2 = createPosition();
            final CoordinateTransformation nedC2 = createOrientation();

            final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, nedC2);
            final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(nedFrame2);

            final ECEFPosition ecefPosition2 = ecefFrame2.getECEFPosition();
            final CoordinateTransformation ecefC2 = ecefFrame2
                    .getCoordinateTransformation();

            final Distance x = new Distance(ecefPosition2.getX(), DistanceUnit.METER);
            final Distance y = new Distance(ecefPosition2.getY(), DistanceUnit.METER);
            final Distance z = new Distance(ecefPosition2.getZ(), DistanceUnit.METER);

            estimator.setEcefPositionAndNedOrientation(x, y, z, nedC2);

            // check
            assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
            if (!estimator.getEcefPosition().equals(ecefPosition2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefPosition().equals(ecefPosition2,
                    LARGE_ABSOLUTE_ERROR));
            assertTrue(estimator.getEcefC().equals(ecefC2, ABSOLUTE_ERROR));

            // Force InvalidSourceAndDestinationFrameTypeException
            try {
                estimator.setEcefPositionAndNedOrientation(x, y, z,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                                FrameType.BODY_FRAME));
                fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
            } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
            }

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testSetEcefPositionAndNedOrientation4()
            throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default values
        final NEDFrame nedFrame1 = new NEDFrame();
        final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame1);

        final NEDPosition nedPosition1 = nedFrame1.getPosition();
        final CoordinateTransformation nedC1 = nedFrame1
                .getCoordinateTransformation();
        final ECEFPosition ecefPosition1 = ecefFrame1.getECEFPosition();
        final CoordinateTransformation ecefC1 = ecefFrame1
                .getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(estimator.getEcefPosition(), ecefPosition1);
        assertEquals(estimator.getEcefC(), ecefC1);

        // set new values
        final NEDPosition nedPosition2 = createPosition();
        final CoordinateTransformation nedC2 = createOrientation();

        final NEDFrame nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame2);

        final ECEFPosition ecefPosition2 = ecefFrame2.getECEFPosition();
        final CoordinateTransformation ecefC2 = ecefFrame2
                .getCoordinateTransformation();

        final double x = ecefPosition2.getX();
        final double y = ecefPosition2.getY();
        final double z = ecefPosition2.getZ();
        final Point3D point = new InhomogeneousPoint3D(x, y, z);

        estimator.setEcefPositionAndNedOrientation(point, nedC2);

        // check
        assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
        assertTrue(estimator.getEcefPosition().equals(ecefPosition2,
                LARGE_ABSOLUTE_ERROR));
        assertTrue(estimator.getEcefC().equals(ecefC2, ABSOLUTE_ERROR));

        // Force InvalidSourceAndDestinationFrameTypeException
        try {
            estimator.setEcefPositionAndNedOrientation(point,
                    new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME,
                            FrameType.BODY_FRAME));
            fail("InvalidSourceAndDestinationFrameTypeException expected but not thrown");
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
        }
    }

    @Test
    public void testIsSetFixKinematicsEnabled() throws LockedException {
        final RandomWalkEstimator estimator = new RandomWalkEstimator();

        // check default value
        assertTrue(estimator.isFixKinematicsEnabled());

        // set new value
        estimator.setFixKinematicsEnabled(false);

        // check
        assertFalse(estimator.isFixKinematicsEnabled());
    }

    @Test
    public void testAddBodyKinematicsWithFixEnabledAndReset()
            throws AlgebraException, LockedException,
            InvalidSourceAndDestinationFrameTypeException,
            RandomWalkEstimationException {
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

        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final CoordinateTransformation nedC = new CoordinateTransformation(
                roll, pitch, yaw, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final CoordinateTransformation ecefC = ecefFrame
                .getCoordinateTransformation();
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final RandomWalkEstimator estimator = new RandomWalkEstimator();
        estimator.setNedPositionAndNedOrientation(nedPosition, nedC);
        estimator.setAccelerationBias(ba);
        estimator.setAccelerationCrossCouplingErrors(ma);
        estimator.setAngularSpeedBias(bg);
        estimator.setAngularSpeedCrossCouplingErrors(mg);
        estimator.setAngularSpeedGDependantCrossBias(gg);
        estimator.setTimeInterval(TIME_INTERVAL_SECONDS);
        estimator.setListener(this);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mBodyKinematicsAdded, 0);
        assertEquals(mReset, 0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isFixKinematicsEnabled());

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

            estimator.addBodyKinematics(measuredKinematics);

            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());
        }

        assertEquals(estimator.getNumberOfProcessedSamples(), N_SAMPLES);
        assertFalse(estimator.isRunning());
        assertEquals(mStart, 1);
        assertEquals(mBodyKinematicsAdded, N_SAMPLES);
        assertEquals(mReset, 0);

        final double accelerometerBiasPsd = estimator.getAccelerometerBiasPSD();
        assertTrue(accelerometerBiasPsd > 0.0);

        final double gyroBiasPsd = estimator.getGyroBiasPSD();
        assertTrue(gyroBiasPsd > 0.0);

        final double positionVariance = estimator.getPositionVariance();
        assertTrue(positionVariance > 0.0);

        final double velocityVariance = estimator.getVelocityVariance();
        assertTrue(velocityVariance > 0.0);

        final double attitudeVariance = estimator.getAttitudeVariance();
        assertTrue(attitudeVariance > 0.0);

        final double positionStd = estimator.getPositionStandardDeviation();
        assertEquals(Math.sqrt(positionVariance), positionStd, 0.0);

        final Distance posStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(positionStd, posStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, posStd1.getUnit());

        final Distance posStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(posStd2);
        assertEquals(posStd1, posStd2);

        final double velocityStd = estimator.getVelocityStandardDeviation();
        assertEquals(Math.sqrt(velocityVariance), velocityStd, 0.0);

        final Speed velStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(velocityStd, velStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velStd1.getUnit());

        final Speed velStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(velStd2);
        assertEquals(velStd1, velStd2);

        final double attitudeStd = estimator.getAttitudeStandardDeviation();
        assertEquals(Math.sqrt(attitudeVariance), attitudeStd, 0.0);

        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(attitudeStd, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());

        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        // check fixed kinematics
        BodyKinematicsFixer fixer = new BodyKinematicsFixer();
        fixer.setAccelerationBias(ba);
        fixer.setAccelerationCrossCouplingErrors(ma);
        fixer.setAngularSpeedBias(bg);
        fixer.setAngularSpeedCrossCouplingErrors(mg);
        fixer.setAngularSpeedGDependantCrossBias(gg);

        final BodyKinematics fixedKinematics1 = fixer.fixAndReturnNew(measuredKinematics);
        final BodyKinematics fixedKinematics2 = estimator.getFixedKinematics();
        final BodyKinematics fixedKinematics3 = new BodyKinematics();
        estimator.getFixedKinematics(fixedKinematics3);

        assertEquals(fixedKinematics1, fixedKinematics2);
        assertEquals(fixedKinematics1, fixedKinematics3);

        assertTrue(estimator.reset());
        assertEquals(mReset, 1);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertFalse(estimator.isRunning());
        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertFalse(estimator.reset());
        assertEquals(mReset, 1);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
    }

    @Test
    public void testAddBodyKinematicsWithFixDisabledAndReset()
            throws AlgebraException, LockedException,
            InvalidSourceAndDestinationFrameTypeException,
            RandomWalkEstimationException {
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

        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final CoordinateTransformation nedC = new CoordinateTransformation(
                roll, pitch, yaw, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame nedFrame = new NEDFrame(nedPosition, nedC);
        final ECEFFrame ecefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(nedFrame);

        final CoordinateTransformation ecefC = ecefFrame
                .getCoordinateTransformation();
        final ECEFPosition ecefPosition = ecefFrame.getECEFPosition();

        final RandomWalkEstimator estimator = new RandomWalkEstimator();
        estimator.setNedPositionAndNedOrientation(nedPosition, nedC);
        estimator.setTimeInterval(TIME_INTERVAL_SECONDS);
        estimator.setListener(this);
        estimator.setFixKinematicsEnabled(false);

        reset();
        assertEquals(mStart, 0);
        assertEquals(mBodyKinematicsAdded, 0);
        assertEquals(mReset, 0);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFixKinematicsEnabled());

        // this is the true body kinematics that should be measured on a perfect
        // sensor if there were no noise or calibration errors at current position
        // and orientations
        final BodyKinematics trueKinematics = ECEFKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                        ecefC, ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition);

        BodyKinematicsFixer fixer = new BodyKinematicsFixer();
        fixer.setAccelerationBias(ba);
        fixer.setAccelerationCrossCouplingErrors(ma);
        fixer.setAngularSpeedBias(bg);
        fixer.setAngularSpeedCrossCouplingErrors(mg);
        fixer.setAngularSpeedGDependantCrossBias(gg);

        final BodyKinematics measuredKinematics = new BodyKinematics();
        // fixed kinematics where bias and cross couplings are compensated but
        // that still contains noise
        final BodyKinematics fixedKinematics1 = new BodyKinematics();
        for (int i = 0; i < N_SAMPLES; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random, measuredKinematics);
            fixer.fix(measuredKinematics, fixedKinematics1);
            estimator.addBodyKinematics(fixedKinematics1);

            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());
        }

        assertEquals(estimator.getNumberOfProcessedSamples(), N_SAMPLES);
        assertFalse(estimator.isRunning());
        assertEquals(mStart, 1);
        assertEquals(mBodyKinematicsAdded, N_SAMPLES);
        assertEquals(mReset, 0);

        final double accelerometerBiasPsd = estimator.getAccelerometerBiasPSD();
        assertTrue(accelerometerBiasPsd > 0.0);

        final double gyroBiasPsd = estimator.getGyroBiasPSD();
        assertTrue(gyroBiasPsd > 0.0);

        final double positionVariance = estimator.getPositionVariance();
        assertTrue(positionVariance > 0.0);

        final double velocityVariance = estimator.getVelocityVariance();
        assertTrue(velocityVariance > 0.0);

        final double attitudeVariance = estimator.getAttitudeVariance();
        assertTrue(attitudeVariance > 0.0);

        final double positionStd = estimator.getPositionStandardDeviation();
        assertEquals(Math.sqrt(positionVariance), positionStd, 0.0);

        final Distance posStd1 = estimator.getPositionStandardDeviationAsDistance();
        assertEquals(positionStd, posStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, posStd1.getUnit());

        final Distance posStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionStandardDeviationAsDistance(posStd2);
        assertEquals(posStd1, posStd2);

        final double velocityStd = estimator.getVelocityStandardDeviation();
        assertEquals(Math.sqrt(velocityVariance), velocityStd, 0.0);

        final Speed velStd1 = estimator.getVelocityStandardDeviationAsSpeed();
        assertEquals(velocityStd, velStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velStd1.getUnit());

        final Speed velStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityStandardDeviationAsSpeed(velStd2);
        assertEquals(velStd1, velStd2);

        final double attitudeStd = estimator.getAttitudeStandardDeviation();
        assertEquals(Math.sqrt(attitudeVariance), attitudeStd, 0.0);

        final Angle attitudeStd1 = estimator.getAttitudeStandardDeviationAsAngle();
        assertEquals(attitudeStd, attitudeStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeStd1.getUnit());

        final Angle attitudeStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeStandardDeviationAsAngle(attitudeStd2);
        assertEquals(attitudeStd1, attitudeStd2);

        // check fixed kinematics
        final BodyKinematics fixedKinematics2 = estimator.getFixedKinematics();
        final BodyKinematics fixedKinematics3 = new BodyKinematics();
        estimator.getFixedKinematics(fixedKinematics3);

        assertEquals(fixedKinematics1, fixedKinematics2);
        assertEquals(fixedKinematics1, fixedKinematics3);

        assertTrue(estimator.reset());
        assertEquals(mReset, 1);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
        assertFalse(estimator.isRunning());
        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeVariance(), 0.0);

        assertFalse(estimator.reset());
        assertEquals(mReset, 1);
        assertEquals(estimator.getNumberOfProcessedSamples(), 0);
    }

    @Override
    public void onStart(final RandomWalkEstimator estimator) {
        checkLocked(estimator);
        mStart++;
    }

    @Override
    public void onBodyKinematicsAdded(final RandomWalkEstimator estimator,
                                      final BodyKinematics measuredKinematics,
                                      final BodyKinematics fixedKinematics) {
        if (mBodyKinematicsAdded == 0) {
            checkLocked(estimator);
        }
        mBodyKinematicsAdded++;
    }

    @Override
    public void onReset(final RandomWalkEstimator estimator) {
        checkLocked(estimator);
        mReset++;
    }

    private void checkLocked(RandomWalkEstimator estimator) {
        assertTrue(estimator.isRunning());
        try {
            estimator.setListener(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
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
            estimator.addBodyKinematics(null);
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        } catch (final RandomWalkEstimationException e) {
            fail("LockedException expected but not thrown");
        }
        try {
            estimator.reset();
            fail("LockedException expected but not thrown");
        } catch (final LockedException ignore) {
        }
    }

    private void reset() {
        mStart = 0;
        mBodyKinematicsAdded = 0;
        mReset = 0;
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

    private NEDPosition createPosition() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(
                randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        return new NEDPosition(latitude, longitude, height);
    }

    private CoordinateTransformation createOrientation() {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());
        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        return new CoordinateTransformation(
                roll, pitch, yaw, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
    }
}
