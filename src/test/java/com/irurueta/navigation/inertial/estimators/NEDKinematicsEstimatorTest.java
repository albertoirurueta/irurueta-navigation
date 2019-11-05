/*
 * Copyright (C) 2019 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.navigation.inertial.estimators;

import com.irurueta.algebra.DecomposerException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.RankDeficientMatrixException;
import com.irurueta.algebra.Utils;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.ECIFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoECIFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.ECIKinematics;
import com.irurueta.navigation.inertial.NEDGravity;
import com.irurueta.navigation.inertial.NEDKinematics;
import com.irurueta.navigation.inertial.RadiiOfCurvature;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class NEDKinematicsEstimatorTest {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double LATITUDE_DEGREES = 41.3825;
    private static final double LONGITUDE_DEGREES = 2.176944;
    private static final double HEIGHT = 0.0;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final double MIN_ANGLE_VARIATION_DEGREES = -5.0;
    private static final double MAX_ANGLE_VARIATION_DEGREES = 5.0;

    private static final double MIN_POSITION_VARIATION_DEGREES = -1e-4;
    private static final double MAX_POSITION_VARIATION_DEGREES = 1e-4;

    private static final double MIN_HEIGHT_VARIATION = -0.5;
    private static final double MAX_HEIGHT_VARIATION = 0.5;

    private static final double MIN_VELOCITY_VARIATION = -0.1;
    private static final double MAX_VELOCITY_VARIATION = 0.1;

    private static final double SCALING_THRESHOLD = 2e-5;
    private static final double ALPHA_THRESHOLD = 1e-8;

    private static final double ABSOLUTE_ERROR = 1e-6;

    private static final double LARGE_ABSOLUTE_ERROR = 1e-1;

    private static final int TIMES = 100;

    @Test
    public void testEstimate()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final NEDFrame oldFrame = createOldNedFrame();

        final NEDFrame frame = createNewNedFrame(oldFrame);

        final CoordinateTransformation c = frame.getCoordinateTransformation();
        final CoordinateTransformation oldC = oldFrame.getCoordinateTransformation();

        final double vn = frame.getVn();
        final double ve = frame.getVe();
        final double vd = frame.getVd();

        final double oldVn = oldFrame.getVn();
        final double oldVe = oldFrame.getVe();
        final double oldVd = oldFrame.getVd();

        final double latitude = frame.getLatitude();
        final double height = frame.getHeight();

        final double oldLatitude = oldFrame.getLatitude();
        final double oldHeight = oldFrame.getHeight();

        final NEDKinematicsEstimator estimator = new NEDKinematicsEstimator();

        final NEDKinematics k1 = new NEDKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight,
                k1);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final NEDKinematics k2 = new NEDKinematics();
        estimator.estimate(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight,
                k2);

        final NEDKinematics k3 = new NEDKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight, k3);

        final NEDKinematics k4 = new NEDKinematics();
        estimator.estimate(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight, k4);

        final NEDKinematics k5 = new NEDKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, vn, ve, vd, latitude, height,
                oldFrame, k5);

        final NEDKinematics k6 = new NEDKinematics();
        estimator.estimate(timeInterval, c, vn, ve, vd, latitude, height,
                oldFrame, k6);

        final NEDKinematics k7 = new NEDKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, frame, oldFrame, k7);

        final NEDKinematics k8 = new NEDKinematics();
        estimator.estimate(timeInterval, frame, oldFrame, k8);

        final Speed speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final Speed speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final Speed speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);

        final NEDKinematics k9 = new NEDKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitude, height,
                oldLatitude, oldHeight, k9);

        final NEDKinematics k10 = new NEDKinematics();
        estimator.estimate(timeInterval, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitude, height,
                oldLatitude, oldHeight, k10);

        final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);

        final NEDKinematics k11 = new NEDKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight,
                k11);

        final NEDKinematics k12 = new NEDKinematics();
        estimator.estimate(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight,
                k12);

        final Distance heightDistance = new Distance(height, DistanceUnit.METER);
        final Distance oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);

        final NEDKinematics k13 = new NEDKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, heightDistance,
                oldLatitude, oldHeightDistance, k13);

        final NEDKinematics k14 = new NEDKinematics();
        estimator.estimate(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, heightDistance,
                oldLatitude, oldHeightDistance, k14);

        final NEDKinematics k15 = new NEDKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, heightDistance,
                oldLatitudeAngle, oldHeightDistance, k15);

        final NEDKinematics k16 = new NEDKinematics();
        estimator.estimate(timeInterval, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, heightDistance,
                oldLatitudeAngle, oldHeightDistance, k16);

        assertEquals(k1, k2);
        assertEquals(k1, k3);
        assertEquals(k1, k4);
        assertEquals(k1, k5);
        assertEquals(k1, k6);
        assertEquals(k1, k7);
        assertEquals(k1, k8);
        assertEquals(k1, k9);
        assertEquals(k1, k10);
        assertEquals(k1, k11);
        assertEquals(k1, k12);
        assertEquals(k1, k13);
        assertEquals(k1, k14);
        assertEquals(k1, k15);
        assertEquals(k1, k16);
    }

    @Test
    public void testEstimateAndReturnNew()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final NEDFrame oldFrame = createOldNedFrame();

        final NEDFrame frame = createNewNedFrame(oldFrame);

        final CoordinateTransformation c = frame.getCoordinateTransformation();
        final CoordinateTransformation oldC = oldFrame.getCoordinateTransformation();

        final double vn = frame.getVn();
        final double ve = frame.getVe();
        final double vd = frame.getVd();

        final double oldVn = oldFrame.getVn();
        final double oldVe = oldFrame.getVe();
        final double oldVd = oldFrame.getVd();

        final double latitude = frame.getLatitude();
        final double height = frame.getHeight();

        final double oldLatitude = oldFrame.getLatitude();
        final double oldHeight = oldFrame.getHeight();

        final NEDKinematicsEstimator estimator = new NEDKinematicsEstimator();

        final NEDKinematics k1 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final NEDKinematics k2 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);

        final NEDKinematics k3 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight);

        final NEDKinematics k4 = estimator.estimateAndReturnNew(
                timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight);

        final NEDKinematics k5 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, vn, ve, vd, latitude, height,
                oldFrame);

        final NEDKinematics k6 = estimator.estimateAndReturnNew(
                timeInterval, c, vn, ve, vd, latitude, height,
                oldFrame);

        final NEDKinematics k7 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, frame, oldFrame);

        final NEDKinematics k8 = estimator.estimateAndReturnNew(
                timeInterval, frame, oldFrame);

        final Speed speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final Speed speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final Speed speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);

        final NEDKinematics k9 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitude, height,
                oldLatitude, oldHeight);

        final NEDKinematics k10 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitude, height,
                oldLatitude, oldHeight);

        final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);

        final NEDKinematics k11 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight);

        final NEDKinematics k12 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight);

        final Distance heightDistance = new Distance(height, DistanceUnit.METER);
        final Distance oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);

        final NEDKinematics k13 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, heightDistance,
                oldLatitude, oldHeightDistance);

        final NEDKinematics k14 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, heightDistance,
                oldLatitude, oldHeightDistance);

        final NEDKinematics k15 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, heightDistance,
                oldLatitudeAngle, oldHeightDistance);

        final NEDKinematics k16 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, heightDistance,
                oldLatitudeAngle, oldHeightDistance);

        assertEquals(k1, k2);
        assertEquals(k1, k3);
        assertEquals(k1, k4);
        assertEquals(k1, k5);
        assertEquals(k1, k6);
        assertEquals(k1, k7);
        assertEquals(k1, k8);
        assertEquals(k1, k9);
        assertEquals(k1, k10);
        assertEquals(k1, k11);
        assertEquals(k1, k12);
        assertEquals(k1, k13);
        assertEquals(k1, k14);
        assertEquals(k1, k15);
        assertEquals(k1, k16);
    }

    @Test
    public void testEstimateKinematics()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, WrongSizeException,
            RankDeficientMatrixException, DecomposerException {

        for (int t = 0; t < TIMES; t++) {
            final NEDFrame oldFrame = createOldNedFrame();

            final NEDFrame frame = createNewNedFrame(oldFrame);

            final CoordinateTransformation c = frame.getCoordinateTransformation();
            final CoordinateTransformation oldC = oldFrame.getCoordinateTransformation();

            final double vn = frame.getVn();
            final double ve = frame.getVe();
            final double vd = frame.getVd();

            final double oldVn = oldFrame.getVn();
            final double oldVe = oldFrame.getVe();
            final double oldVd = oldFrame.getVd();

            final double latitude = frame.getLatitude();
            final double height = frame.getHeight();

            final double oldLatitude = oldFrame.getLatitude();
            final double oldHeight = oldFrame.getHeight();

            final NEDKinematics k1 = new NEDKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC,
                    vn, ve, vd, oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight,
                    k1);

            final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
            final NEDKinematics k2 = new NEDKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight,
                    k2);

            final NEDKinematics k3 = new NEDKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, frame, oldC,
                    oldVn, oldVe, oldVd, oldLatitude, oldHeight, k3);

            final NEDKinematics k4 = new NEDKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, frame, oldC,
                    oldVn, oldVe, oldVd, oldLatitude, oldHeight, k4);

            final NEDKinematics k5 = new NEDKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c,
                    vn, ve, vd, latitude, height, oldFrame, k5);

            final NEDKinematics k6 = new NEDKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, vn, ve, vd,
                    latitude, height, oldFrame, k6);

            final NEDKinematics k7 = new NEDKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, frame,
                    oldFrame, k7);

            final NEDKinematics k8 = new NEDKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, frame, oldFrame, k8);

            final Speed speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
            final Speed speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
            final Speed speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

            final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
            final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
            final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);

            final NEDKinematics k9 = new NEDKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC,
                    speedN, speedE, speedD, oldSpeedN, oldSpeedE, oldSpeedD,
                    latitude, height, oldLatitude, oldHeight, k9);

            final NEDKinematics k10 = new NEDKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, oldC,
                    speedN, speedE, speedD, oldSpeedN, oldSpeedE, oldSpeedD,
                    latitude, height, oldLatitude, oldHeight, k10);

            final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
            final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);

            final NEDKinematics k11 = new NEDKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC,
                    vn, ve, vd, oldVn, oldVe, oldVd, latitudeAngle, height,
                    oldLatitudeAngle, oldHeight, k11);

            final NEDKinematics k12 = new NEDKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight,
                    k12);

            final Distance heightDistance = new Distance(height, DistanceUnit.METER);
            final Distance oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);

            final NEDKinematics k13 = new NEDKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC,
                    vn, ve, vd, oldVn, oldVe, oldVd, latitude, heightDistance,
                    oldLatitude, oldHeightDistance, k13);

            final NEDKinematics k14 = new NEDKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, latitude, heightDistance,
                    oldLatitude, oldHeightDistance, k14);

            final NEDKinematics k15 = new NEDKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC,
                    speedN, speedE, speedD, oldSpeedN, oldSpeedE, oldSpeedD,
                    latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance, k15);

            final NEDKinematics k16 = new NEDKinematics();
            NEDKinematicsEstimator.estimateKinematics(timeInterval, c, oldC,
                    speedN, speedE, speedD, oldSpeedN, oldSpeedE, oldSpeedD,
                    latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance, k16);

            assertEquals(k1, k2);
            assertEquals(k1, k3);
            assertEquals(k1, k4);
            assertEquals(k1, k5);
            assertEquals(k1, k6);
            assertEquals(k1, k7);
            assertEquals(k1, k8);
            assertEquals(k1, k9);
            assertEquals(k1, k10);
            assertEquals(k1, k11);
            assertEquals(k1, k12);
            assertEquals(k1, k13);
            assertEquals(k1, k14);
            assertEquals(k1, k15);
            assertEquals(k1, k16);

            final NEDKinematics k = estimateKinematics(TIME_INTERVAL_SECONDS,
                    c, oldC, vn, ve, vd, oldVn, oldVe, oldVd, latitude, height,
                    oldLatitude, oldHeight);

            assertTrue(k1.equals(k, ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testEstimateKinematicsAndReturnNew()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, WrongSizeException, RankDeficientMatrixException, DecomposerException {

        for (int t = 0; t < TIMES; t++) {
            final NEDFrame oldFrame = createOldNedFrame();

            final NEDFrame frame = createNewNedFrame(oldFrame);

            final CoordinateTransformation c = frame.getCoordinateTransformation();
            final CoordinateTransformation oldC = oldFrame.getCoordinateTransformation();

            final double vn = frame.getVn();
            final double ve = frame.getVe();
            final double vd = frame.getVd();

            final double oldVn = oldFrame.getVn();
            final double oldVe = oldFrame.getVe();
            final double oldVd = oldFrame.getVd();

            final double latitude = frame.getLatitude();
            final double height = frame.getHeight();

            final double oldLatitude = oldFrame.getLatitude();
            final double oldHeight = oldFrame.getHeight();

            final NEDKinematics k1 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                            TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                            oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);

            final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
            final NEDKinematics k2 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                            timeInterval, c, oldC, vn, ve, vd,
                            oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);

            final NEDKinematics k3 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                            TIME_INTERVAL_SECONDS, frame, oldC, oldVn, oldVe, oldVd,
                            oldLatitude, oldHeight);

            final NEDKinematics k4 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                            timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                            oldLatitude, oldHeight);

            final NEDKinematics k5 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                            TIME_INTERVAL_SECONDS, c, vn, ve, vd, latitude, height,
                            oldFrame);

            final NEDKinematics k6 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                            timeInterval, c, vn, ve, vd, latitude, height,
                            oldFrame);

            final NEDKinematics k7 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                            TIME_INTERVAL_SECONDS, frame, oldFrame);

            final NEDKinematics k8 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                            timeInterval, frame, oldFrame);

            final Speed speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
            final Speed speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
            final Speed speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

            final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
            final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
            final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);

            final NEDKinematics k9 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                            TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                            oldSpeedN, oldSpeedE, oldSpeedD, latitude, height,
                            oldLatitude, oldHeight);

            final NEDKinematics k10 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                            timeInterval, c, oldC, speedN, speedE, speedD,
                            oldSpeedN, oldSpeedE, oldSpeedD, latitude, height,
                            oldLatitude, oldHeight);

            final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
            final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);

            final NEDKinematics k11 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                            TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                            oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight);

            final NEDKinematics k12 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                            timeInterval, c, oldC, vn, ve, vd,
                            oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight);

            final Distance heightDistance = new Distance(height, DistanceUnit.METER);
            final Distance oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);

            final NEDKinematics k13 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                            TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                            oldVn, oldVe, oldVd, latitude, heightDistance,
                            oldLatitude, oldHeightDistance);

            final NEDKinematics k14 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                            timeInterval, c, oldC, vn, ve, vd,
                            oldVn, oldVe, oldVd, latitude, heightDistance,
                            oldLatitude, oldHeightDistance);

            final NEDKinematics k15 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                            TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                            oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, heightDistance,
                            oldLatitudeAngle, oldHeightDistance);

            final NEDKinematics k16 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                            timeInterval, c, oldC, speedN, speedE, speedD,
                            oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, heightDistance,
                            oldLatitudeAngle, oldHeightDistance);

            assertEquals(k1, k2);
            assertEquals(k1, k3);
            assertEquals(k1, k4);
            assertEquals(k1, k5);
            assertEquals(k1, k6);
            assertEquals(k1, k7);
            assertEquals(k1, k8);
            assertEquals(k1, k9);
            assertEquals(k1, k10);
            assertEquals(k1, k11);
            assertEquals(k1, k12);
            assertEquals(k1, k13);
            assertEquals(k1, k14);
            assertEquals(k1, k15);
            assertEquals(k1, k16);

            final NEDKinematics k = estimateKinematics(TIME_INTERVAL_SECONDS,
                    c, oldC, vn, ve, vd, oldVn, oldVe, oldVd, latitude, height,
                    oldLatitude, oldHeight);

            assertTrue(k1.equals(k, ABSOLUTE_ERROR));
        }
    }

    @Test(expected = IllegalArgumentException.class)
    public void testEstimateKinematicsWhenNegativeIntervalThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final NEDFrame oldFrame = createOldNedFrame();

        final NEDFrame frame = createNewNedFrame(oldFrame);

        final CoordinateTransformation c = frame.getCoordinateTransformation();
        final CoordinateTransformation oldC = oldFrame.getCoordinateTransformation();

        final double vn = frame.getVn();
        final double ve = frame.getVe();
        final double vd = frame.getVd();

        final double oldVn = oldFrame.getVn();
        final double oldVe = oldFrame.getVe();
        final double oldVd = oldFrame.getVd();

        final double latitude = frame.getLatitude();
        final double height = frame.getHeight();

        final double oldLatitude = oldFrame.getLatitude();
        final double oldHeight = oldFrame.getHeight();

        NEDKinematicsEstimator.estimateKinematicsAndReturnNew(
                -1.0, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);
    }

    @Test(expected = IllegalArgumentException.class)
    public void testEstimateKinematicsWhenInvalidCoordinateTransformationThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final NEDFrame oldFrame = createOldNedFrame();

        final NEDFrame frame = createNewNedFrame(oldFrame);

        final CoordinateTransformation c = frame.getCoordinateTransformation();
        final CoordinateTransformation oldC = oldFrame.getCoordinateTransformation();

        final double vn = frame.getVn();
        final double ve = frame.getVe();
        final double vd = frame.getVd();

        final double oldVn = oldFrame.getVn();
        final double oldVe = oldFrame.getVe();
        final double oldVd = oldFrame.getVd();

        final double latitude = frame.getLatitude();
        final double height = frame.getHeight();

        final double oldLatitude = oldFrame.getLatitude();
        final double oldHeight = oldFrame.getHeight();

        c.setDestinationType(FrameType.BODY_FRAME);
        NEDKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);
    }

    @Test(expected = IllegalArgumentException.class)
    public void testEstimateKinematicsWhenInvalidOldCoordinateTransformationThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final NEDFrame oldFrame = createOldNedFrame();

        final NEDFrame frame = createNewNedFrame(oldFrame);

        final CoordinateTransformation c = frame.getCoordinateTransformation();
        final CoordinateTransformation oldC = oldFrame.getCoordinateTransformation();

        final double vn = frame.getVn();
        final double ve = frame.getVe();
        final double vd = frame.getVd();

        final double oldVn = oldFrame.getVn();
        final double oldVe = oldFrame.getVe();
        final double oldVd = oldFrame.getVd();

        final double latitude = frame.getLatitude();
        final double height = frame.getHeight();

        final double oldLatitude = oldFrame.getLatitude();
        final double oldHeight = oldFrame.getHeight();

        oldC.setDestinationType(FrameType.BODY_FRAME);
        NEDKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);
    }

    @Test
    public void testEstimateKinematicsWhenZeroTimeIntervalReturnsZeroValues()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, WrongSizeException,
            RankDeficientMatrixException, DecomposerException {

        final NEDFrame oldFrame = createOldNedFrame();

        final NEDFrame frame = createNewNedFrame(oldFrame);

        final CoordinateTransformation c = frame.getCoordinateTransformation();
        final CoordinateTransformation oldC = oldFrame.getCoordinateTransformation();

        final double vn = frame.getVn();
        final double ve = frame.getVe();
        final double vd = frame.getVd();

        final double oldVn = oldFrame.getVn();
        final double oldVe = oldFrame.getVe();
        final double oldVd = oldFrame.getVd();

        final double latitude = frame.getLatitude();
        final double height = frame.getHeight();

        final double oldLatitude = oldFrame.getLatitude();
        final double oldHeight = oldFrame.getHeight();

        final NEDKinematics k = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(
                0.0, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight);

        assertEquals(k.getFx(), 0.0, 0.0);
        assertEquals(k.getFy(), 0.0, 0.0);
        assertEquals(k.getFz(), 0.0, 0.0);

        assertEquals(k.getAngularRateX(), 0.0, 0.0);
        assertEquals(k.getAngularRateY(), 0.0, 0.0);
        assertEquals(k.getAngularRateZ(), 0.0, 0.0);

        final NEDKinematics k2 = estimateKinematics(0.0, c, oldC,
                vn, ve, vd, oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);

        assertTrue(k2.equals(k, 0.0));
    }

    @Test
    public void testCompareNEDAndECIKinematics() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {
        for (int t = 0; t < TIMES; t++) {
            final NEDFrame oldNedFrame = createOldNedFrame();
            final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(oldNedFrame);
            final ECIFrame oldEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(
                    TIME_INTERVAL_SECONDS, oldEcefFrame);

            final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
            final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(newNedFrame);
            final ECIFrame newEciFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(
                    TIME_INTERVAL_SECONDS, newEcefFrame);

            final NEDKinematics nedK = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, newNedFrame, oldNedFrame);

            final ECIKinematics eciK = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, newEciFrame, oldEciFrame);

            final double nedSpecificForceNorm = nedK.getSpecificForceNorm();
            final double eciSpecificForceNorm = eciK.getSpecificForceNorm();

            final double nedAngularRateNorm = nedK.getAngularRateNorm();
            final double eciAngularRateNorm = eciK.getAngularRateNorm();

            assertEquals(eciSpecificForceNorm, nedSpecificForceNorm, LARGE_ABSOLUTE_ERROR);
            assertEquals(eciAngularRateNorm, nedAngularRateNorm, LARGE_ABSOLUTE_ERROR);

            assertEquals(eciK.getFx(), nedK.getFx(), LARGE_ABSOLUTE_ERROR);
            assertEquals(eciK.getFy(), nedK.getFy(), LARGE_ABSOLUTE_ERROR);
            assertEquals(eciK.getFz(), nedK.getFz(), LARGE_ABSOLUTE_ERROR);
            assertEquals(eciK.getAngularRateX(), nedK.getAngularRateX(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(eciK.getAngularRateY(), nedK.getAngularRateY(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(eciK.getAngularRateZ(), nedK.getAngularRateZ(),
                    LARGE_ABSOLUTE_ERROR);
        }
    }

    private NEDFrame createOldNedFrame()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double vn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double ve = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double vd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Quaternion q = new Quaternion(roll, pitch, yaw);

        final Matrix m = q.asInhomogeneousMatrix();
        final CoordinateTransformation c = new CoordinateTransformation(
                m, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final double latitude = Math.toRadians(LATITUDE_DEGREES);
        final double longitude = Math.toRadians(LONGITUDE_DEGREES);
        return new NEDFrame(latitude, longitude, HEIGHT, vn, ve, vd, c);
    }

    private NEDFrame createNewNedFrame(final NEDFrame oldFrame)
            throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException {

        final double oldLatitude = oldFrame.getLatitude();
        final double oldLongitude = oldFrame.getLongitude();
        final double oldHeight = oldFrame.getHeight();

        final double oldVn = oldFrame.getVn();
        final double oldVe = oldFrame.getVe();
        final double oldVd = oldFrame.getVd();

        final CoordinateTransformation oldC = oldFrame
                .getCoordinateTransformation();

        final double oldRoll = oldC.getRollEulerAngle();
        final double oldPitch = oldC.getPitchEulerAngle();
        final double oldYaw = oldC.getYawEulerAngle();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double latitudeVariation = Math.toRadians(randomizer.nextDouble(
                MIN_POSITION_VARIATION_DEGREES,
                MAX_POSITION_VARIATION_DEGREES));
        final double longitudeVariation = Math.toRadians(randomizer.nextDouble(
                MIN_POSITION_VARIATION_DEGREES,
                MAX_POSITION_VARIATION_DEGREES));
        final double heightVariation = randomizer.nextDouble(
                MIN_HEIGHT_VARIATION, MAX_HEIGHT_VARIATION);

        final double vnVariation = randomizer.nextDouble(MIN_VELOCITY_VARIATION,
                MAX_VELOCITY_VARIATION);
        final double veVariation = randomizer.nextDouble(MIN_VELOCITY_VARIATION,
                MAX_VELOCITY_VARIATION);
        final double vdVariation = randomizer.nextDouble(MIN_VELOCITY_VARIATION,
                MAX_VELOCITY_VARIATION);

        final double rollVariation = Math.toRadians(randomizer.nextDouble(
                MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES));
        final double pitchVariation = Math.toRadians(randomizer.nextDouble(
                MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES));
        final double yawVariation = Math.toRadians(randomizer.nextDouble(
                MIN_ANGLE_VARIATION_DEGREES, MAX_ANGLE_VARIATION_DEGREES));

        final double latitude = oldLatitude + latitudeVariation;
        final double longitude = oldLongitude + longitudeVariation;
        final double height = oldHeight + heightVariation;

        final double vn = oldVn + vnVariation;
        final double ve = oldVe + veVariation;
        final double vd = oldVd + vdVariation;

        final double roll = oldRoll + rollVariation;
        final double pitch = oldPitch + pitchVariation;
        final double yaw = oldYaw + yawVariation;

        final Quaternion q = new Quaternion(roll, pitch, yaw);

        final Matrix m = q.asInhomogeneousMatrix();
        final CoordinateTransformation c = new CoordinateTransformation(
                m, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        return new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
    }

    private static NEDKinematics estimateKinematics(final double timeInterval,
                                                    final CoordinateTransformation c,
                                                    final CoordinateTransformation oldC,
                                                    final double vn,
                                                    final double ve,
                                                    final double vd,
                                                    final double oldVn,
                                                    final double oldVe,
                                                    final double oldVd,
                                                    final double latitude,
                                                    final double height,
                                                    final double oldLatitude,
                                                    final double oldHeight)
            throws WrongSizeException, RankDeficientMatrixException, DecomposerException {

        if (timeInterval > 0.0) {
            final Matrix oldVebn = new Matrix(3, 1);
            oldVebn.setElementAtIndex(0, oldVn);
            oldVebn.setElementAtIndex(1, oldVe);
            oldVebn.setElementAtIndex(2, oldVd);

            final Matrix vEbn = new Matrix(3, 1);
            vEbn.setElementAtIndex(0, vn);
            vEbn.setElementAtIndex(1, ve);
            vEbn.setElementAtIndex(2, vd);

            final Matrix cBn = c.getMatrix();
            final Matrix oldCbn = oldC.getMatrix();

            final double omegaIe = Constants.EARTH_ROTATION_RATE;

            // From (2.123), determine the angular rate of the ECEF frame with respect
            // the ECI frame, resolved about NED
            Matrix tmp = new Matrix(3, 1);
            tmp.setElementAtIndex(0, Math.cos(oldLatitude));
            tmp.setElementAtIndex(1, 0.0);
            tmp.setElementAtIndex(2, -Math.sin(oldLatitude));

            final Matrix omegaIen = tmp.multiplyByScalarAndReturnNew(omegaIe);

            // From (5.44), determine the angular rate of the NED frame with respect
            // the ECEF frame, resolved about NED
            final RadiiOfCurvature oldRadiiOfCurvature = RadiiOfCurvatureEstimator.
                    estimateRadiiOfCurvatureAndReturnNew(oldLatitude);
            final double oldRn = oldRadiiOfCurvature.getRn();
            final double oldRe = oldRadiiOfCurvature.getRe();

            final RadiiOfCurvature radiiOfCurvature = RadiiOfCurvatureEstimator
                    .estimateRadiiOfCurvatureAndReturnNew(latitude);
            final double rn = radiiOfCurvature.getRn();
            final double re = radiiOfCurvature.getRe();

            final Matrix oldOmegaEnN = new Matrix(3, 1);
            oldOmegaEnN.setElementAtIndex(0,
                    oldVebn.getElementAtIndex(1) / (oldRe + oldHeight));
            oldOmegaEnN.setElementAtIndex(1,
                    -oldVebn.getElementAtIndex(0) / (oldRn + oldHeight));
            oldOmegaEnN.setElementAtIndex(2,
                    -oldVebn.getElementAtIndex(1) * Math.tan(oldLatitude) / (oldRe + oldHeight));

            final Matrix omegaEnN = new Matrix(3, 1);
            omegaEnN.setElementAtIndex(0,
                    vEbn.getElementAtIndex(1) / (re + height));
            omegaEnN.setElementAtIndex(1,
                    -vEbn.getElementAtIndex(0) / (rn + height));
            omegaEnN.setElementAtIndex(2,
                    -vEbn.getElementAtIndex(1) * Math.tan(latitude) / (re + height));

            // Obtain coordinate transformation matrix from the old attitude with respect
            // an inertial frame to the new using (5.77)

            //cOldNew = cBn' * (I - timeInterval*skew(omegaIen + 0.5*omegaEnN + 0.5*oldOmegaEnN)) * oldCbn
            final Matrix cOldNew = cBn.transposeAndReturnNew().multiplyAndReturnNew(
                    Matrix.identity(3, 3).subtractAndReturnNew(
                            Utils.skewMatrix(omegaIen.addAndReturnNew(
                                    omegaEnN.multiplyByScalarAndReturnNew(0.5)).addAndReturnNew(
                                    oldOmegaEnN.multiplyByScalarAndReturnNew(0.5)))
                                    .multiplyByScalarAndReturnNew(timeInterval))
                            .multiplyAndReturnNew(oldCbn));

            // Calculate the approximate angular rate with respect an inertial frame
            final Matrix alphaIbb = new Matrix(3, 1);
            alphaIbb.setElementAtIndex(0, 0.5 * (
                    cOldNew.getElementAt(1, 2) - cOldNew.getElementAt(2, 1)));
            alphaIbb.setElementAtIndex(1, 0.5 * (
                    cOldNew.getElementAt(2, 0) - cOldNew.getElementAt(0, 2)));
            alphaIbb.setElementAtIndex(2, 0.5 * (
                    cOldNew.getElementAt(0, 1) - cOldNew.getElementAt(1, 0)));

            // Calculate and apply the scaling factor
            final double temp = Math.acos(0.5 * (cOldNew.getElementAt(0, 0) +
                    cOldNew.getElementAt(1, 1) +
                    cOldNew.getElementAt(2, 2) - 1.0));
            if (temp > SCALING_THRESHOLD) {
                // scaling is 1 if temp is less than this
                alphaIbb.multiplyByScalar(temp / Math.sin(temp));
            }

            // Calculate the angular rate
            final Matrix omegaIbb = alphaIbb.multiplyByScalarAndReturnNew(
                    1.0 / timeInterval);

            // Calculate the specific force resolved about ECEF-frame axes
            // From (5.54),
            final NEDGravity gravity = NEDGravityEstimator
                    .estimateGravityAndReturnNew(oldLatitude, oldHeight);
            final Matrix g = gravity.asMatrix();
            final Matrix fIbn = vEbn.subtractAndReturnNew(oldVebn);
            fIbn.multiplyByScalar(1.0 / timeInterval);
            fIbn.subtract(g);
            fIbn.add(Utils.skewMatrix(oldOmegaEnN.addAndReturnNew(
                    omegaIen.multiplyByScalarAndReturnNew(2.0)))
                    .multiplyAndReturnNew(oldVebn));

            // Calculate the average body-to-NED coordinate transformation
            // matrix over the update interval using (5.84) and (5.86)
            final double magAlpha = Utils.normF(alphaIbb);
            final Matrix AlphaIbb = Utils.skewMatrix(alphaIbb);
            final Matrix aveCbn;
            if (magAlpha > ALPHA_THRESHOLD) {
                //aveCbn = oldCbn * (I + (1 - cos(magAlpha))/magAlpha^2*AlphaIbb +
                //  (1 - sin(magAlpha)/magAlpha)/magAlpha^2*AlphaIbb*AlphaIbb)
                //  -0.5 * skew(oldOmegaEnn + omegaIen)*oldCbn
                aveCbn = oldCbn.multiplyAndReturnNew(
                        Matrix.identity(3, 3).addAndReturnNew(
                                AlphaIbb.multiplyByScalarAndReturnNew(
                                        (1.0 - Math.cos(magAlpha)) / Math.pow(magAlpha, 2.0))
                        ).addAndReturnNew(AlphaIbb.multiplyByScalarAndReturnNew(
                                (1.0 - Math.sin(magAlpha) / magAlpha) / Math.pow(magAlpha, 2.0))
                                .multiplyAndReturnNew(AlphaIbb))).subtractAndReturnNew(
                        Utils.skewMatrix(oldOmegaEnN.addAndReturnNew(omegaIen))
                                .multiplyByScalarAndReturnNew(0.5)
                                .multiplyAndReturnNew(oldCbn));
            } else {
                aveCbn = oldCbn.subtractAndReturnNew(
                        Utils.skewMatrix(oldOmegaEnN.addAndReturnNew(omegaIen))
                                .multiplyByScalarAndReturnNew(0.5).multiplyAndReturnNew(oldCbn));
            }

            // Transform specific force to body-frame resolving axes using (5.81)
            final Matrix fIbb = Utils.inverse(aveCbn).multiplyAndReturnNew(fIbn);

            final double fx = fIbb.getElementAtIndex(0);
            final double fy = fIbb.getElementAtIndex(1);
            final double fz = fIbb.getElementAtIndex(2);

            final double angularRateX = omegaIbb.getElementAtIndex(0);
            final double angularRateY = omegaIbb.getElementAtIndex(1);
            final double angularRateZ = omegaIbb.getElementAtIndex(2);

            return new NEDKinematics(fx, fy, fz,
                    angularRateX, angularRateY, angularRateZ);

        } else {
            // If time interval is zero, set angular rate and specific force to zero
            return new NEDKinematics();
        }
    }
}
