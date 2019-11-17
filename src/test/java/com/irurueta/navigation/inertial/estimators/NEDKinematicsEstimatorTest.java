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
import com.irurueta.navigation.inertial.NEDGravity;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;
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
        final double longitude = frame.getLongitude();
        final double height = frame.getHeight();

        final double oldLatitude = oldFrame.getLatitude();
        final double oldLongitude = oldFrame.getLongitude();
        final double oldHeight = oldFrame.getHeight();

        final NEDKinematicsEstimator estimator = new NEDKinematicsEstimator();

        final BodyKinematics k1 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight,
                k1);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final BodyKinematics k2 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight,
                k2);

        final NEDVelocity velocity = new NEDVelocity(vn, ve, vd);
        final NEDVelocity oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final BodyKinematics k3 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS,
                c, oldC, velocity, oldVelocity, latitude, height,
                oldLatitude, oldHeight, k3);

        final BodyKinematics k4 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, velocity, oldVelocity,
                latitude, height, oldLatitude, oldHeight, k4);

        final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final Distance heightDistance = new Distance(height, DistanceUnit.METER);
        final Distance oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);
        final BodyKinematics k5 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity,
                latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance,
                k5);

        final BodyKinematics k6 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, velocity, oldVelocity,
                latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance,
                k6);

        final NEDPosition position = new NEDPosition(latitude, longitude, height);
        final NEDPosition oldPosition = new NEDPosition(oldLatitude, oldLongitude,
                oldHeight);
        final BodyKinematics k7 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, position, oldPosition, k7);

        final BodyKinematics k8 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                position, oldPosition, k8);

        final Speed speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final Speed speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final Speed speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final BodyKinematics k9 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, position, oldPosition, k9);

        final BodyKinematics k10 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, position, oldPosition, k10);

        final BodyKinematics k11 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity,
                position, oldPosition, k11);

        final BodyKinematics k12 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, velocity, oldVelocity,
                position, oldPosition, k12);

        final BodyKinematics k13 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight, k13);

        final BodyKinematics k14 = new BodyKinematics();
        estimator.estimate(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight, k14);

        final BodyKinematics k15 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity, oldLatitude,
                oldHeight, k15);

        final BodyKinematics k16 = new BodyKinematics();
        estimator.estimate(timeInterval, frame, oldC, oldVelocity, oldLatitude,
                oldHeight, k16);

        final BodyKinematics k17 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity,
                oldLatitude, oldHeight, k17);

        final BodyKinematics k18 = new BodyKinematics();
        estimator.estimate(timeInterval, frame, oldC, oldVelocity, oldLatitude,
                oldHeight, k18);

        final BodyKinematics k19 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, frame, oldC, oldVn, oldVe, oldVd,
                oldPosition, k19);

        final BodyKinematics k20 = new BodyKinematics();
        estimator.estimate(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldPosition, k20);

        final BodyKinematics k21 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, frame, oldC, oldSpeedN,
                oldSpeedE, oldSpeedD, oldPosition, k21);

        final BodyKinematics k22 = new BodyKinematics();
        estimator.estimate(timeInterval, frame, oldC, oldSpeedN, oldSpeedE,
                oldSpeedD, oldPosition, k22);

        final BodyKinematics k23 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity,
                oldPosition, k23);

        final BodyKinematics k24 = new BodyKinematics();
        estimator.estimate(timeInterval, frame, oldC, oldVelocity, oldPosition,
                k24);

        final BodyKinematics k25 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, vn, ve, vd, latitude, height,
                oldFrame, k25);

        final BodyKinematics k26 = new BodyKinematics();
        estimator.estimate(timeInterval, c, vn, ve, vd, latitude, height,
                oldFrame, k26);

        final BodyKinematics k27 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, velocity, latitude, height,
                oldFrame, k27);

        final BodyKinematics k28 = new BodyKinematics();
        estimator.estimate(timeInterval, c, velocity, latitude, height, oldFrame,
                k28);

        final BodyKinematics k29 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, velocity, latitudeAngle,
                heightDistance, oldFrame, k29);

        final BodyKinematics k30 = new BodyKinematics();
        estimator.estimate(timeInterval, c, velocity, latitudeAngle, heightDistance,
                oldFrame, k30);

        final BodyKinematics k31 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, vn, ve, vd, position, oldFrame,
                k31);

        final BodyKinematics k32 = new BodyKinematics();
        estimator.estimate(timeInterval, c, vn, ve, vd, position, oldFrame, k32);

        final BodyKinematics k33 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, speedN, speedE, speedD,
                position, oldFrame, k33);

        final BodyKinematics k34 = new BodyKinematics();
        estimator.estimate(timeInterval, c, speedN, speedE, speedD, position,
                oldFrame, k34);

        final BodyKinematics k35 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, velocity, position, oldFrame,
                k35);

        final BodyKinematics k36 = new BodyKinematics();
        estimator.estimate(timeInterval, c, velocity, position, oldFrame, k36);

        final BodyKinematics k37 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, frame, oldFrame, k37);

        final BodyKinematics k38 = new BodyKinematics();
        estimator.estimate(timeInterval, frame, oldFrame, k38);

        final BodyKinematics k39 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitude, height,
                oldLatitude, oldHeight, k39);

        final BodyKinematics k40 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitude, height,
                oldLatitude, oldHeight, k40);

        final BodyKinematics k41 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight,
                k41);

        final BodyKinematics k42 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight,
                k42);

        final BodyKinematics k43 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, heightDistance,
                oldLatitude, oldHeightDistance, k43);

        final BodyKinematics k44 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, heightDistance,
                oldLatitude, oldHeightDistance, k44);

        final BodyKinematics k45 = new BodyKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, heightDistance,
                oldLatitudeAngle, oldHeightDistance, k45);

        final BodyKinematics k46 = new BodyKinematics();
        estimator.estimate(timeInterval, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, heightDistance,
                oldLatitudeAngle, oldHeightDistance, k46);

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
        assertEquals(k1, k17);
        assertEquals(k1, k18);
        assertEquals(k1, k19);
        assertEquals(k1, k20);
        assertEquals(k1, k21);
        assertEquals(k1, k22);
        assertEquals(k1, k23);
        assertEquals(k1, k24);
        assertEquals(k1, k25);
        assertEquals(k1, k26);
        assertEquals(k1, k27);
        assertEquals(k1, k28);
        assertEquals(k1, k29);
        assertEquals(k1, k30);
        assertEquals(k1, k31);
        assertEquals(k1, k32);
        assertEquals(k1, k33);
        assertEquals(k1, k34);
        assertEquals(k1, k35);
        assertEquals(k1, k36);
        assertEquals(k1, k37);
        assertEquals(k1, k38);
        assertEquals(k1, k39);
        assertEquals(k1, k40);
        assertEquals(k1, k41);
        assertEquals(k1, k42);
        assertEquals(k1, k43);
        assertEquals(k1, k44);
        assertEquals(k1, k45);
        assertEquals(k1, k46);
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
        final double longitude = frame.getLongitude();
        final double height = frame.getHeight();

        final double oldLatitude = oldFrame.getLatitude();
        final double oldLongitude = oldFrame.getLongitude();
        final double oldHeight = oldFrame.getHeight();

        final NEDKinematicsEstimator estimator = new NEDKinematicsEstimator();

        final BodyKinematics k1 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final BodyKinematics k2 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);

        final NEDVelocity velocity = new NEDVelocity(vn, ve, vd);
        final NEDVelocity oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final BodyKinematics k3 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                c, oldC, velocity, oldVelocity, latitude, height,
                oldLatitude, oldHeight);

        final BodyKinematics k4 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC, velocity, oldVelocity,
                latitude, height, oldLatitude, oldHeight);

        final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final Distance heightDistance = new Distance(height, DistanceUnit.METER);
        final Distance oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);
        final BodyKinematics k5 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity,
                latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance);

        final BodyKinematics k6 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC, velocity, oldVelocity,
                latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance);

        final NEDPosition position = new NEDPosition(latitude, longitude, height);
        final NEDPosition oldPosition = new NEDPosition(oldLatitude, oldLongitude,
                oldHeight);
        final BodyKinematics k7 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, position, oldPosition);

        final BodyKinematics k8 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                position, oldPosition);

        final Speed speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final Speed speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final Speed speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final BodyKinematics k9 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, position, oldPosition);

        final BodyKinematics k10 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, position, oldPosition);

        final BodyKinematics k11 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity,
                position, oldPosition);

        final BodyKinematics k12 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC, velocity, oldVelocity,
                position, oldPosition);

        final BodyKinematics k13 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight);

        final BodyKinematics k14 = estimator.estimateAndReturnNew(
                timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight);

        final BodyKinematics k15 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity, oldLatitude,
                oldHeight);

        final BodyKinematics k16 = estimator.estimateAndReturnNew(
                timeInterval, frame, oldC, oldVelocity, oldLatitude,
                oldHeight);

        final BodyKinematics k17 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity,
                oldLatitude, oldHeight);

        final BodyKinematics k18 = estimator.estimateAndReturnNew(
                timeInterval, frame, oldC, oldVelocity, oldLatitude,
                oldHeight);

        final BodyKinematics k19 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, frame, oldC, oldVn, oldVe, oldVd,
                oldPosition);

        final BodyKinematics k20 = estimator.estimateAndReturnNew(
                timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldPosition);

        final BodyKinematics k21 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, frame, oldC, oldSpeedN,
                oldSpeedE, oldSpeedD, oldPosition);

        final BodyKinematics k22 = estimator.estimateAndReturnNew(
                timeInterval, frame, oldC, oldSpeedN, oldSpeedE,
                oldSpeedD, oldPosition);

        final BodyKinematics k23 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity,
                oldPosition);

        final BodyKinematics k24 = estimator.estimateAndReturnNew(
                timeInterval, frame, oldC, oldVelocity, oldPosition);

        final BodyKinematics k25 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, vn, ve, vd, latitude, height,
                oldFrame);

        final BodyKinematics k26 = estimator.estimateAndReturnNew(
                timeInterval, c, vn, ve, vd, latitude, height,
                oldFrame);

        final BodyKinematics k27 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, velocity, latitude, height,
                oldFrame);

        final BodyKinematics k28 = estimator.estimateAndReturnNew(
                timeInterval, c, velocity, latitude, height, oldFrame);

        final BodyKinematics k29 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, velocity, latitudeAngle,
                heightDistance, oldFrame);

        final BodyKinematics k30 = estimator.estimateAndReturnNew(
                timeInterval, c, velocity, latitudeAngle, heightDistance,
                oldFrame);

        final BodyKinematics k31 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, vn, ve, vd, position, oldFrame);

        final BodyKinematics k32 = estimator.estimateAndReturnNew(
                timeInterval, c, vn, ve, vd, position, oldFrame);

        final BodyKinematics k33 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, speedN, speedE, speedD,
                position, oldFrame);

        final BodyKinematics k34 = estimator.estimateAndReturnNew(
                timeInterval, c, speedN, speedE, speedD, position,
                oldFrame);

        final BodyKinematics k35 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, velocity, position, oldFrame);

        final BodyKinematics k36 = estimator.estimateAndReturnNew(
                timeInterval, c, velocity, position, oldFrame);

        final BodyKinematics k37 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, frame, oldFrame);

        final BodyKinematics k38 = estimator.estimateAndReturnNew(
                timeInterval, frame, oldFrame);

        final BodyKinematics k39 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitude, height,
                oldLatitude, oldHeight);

        final BodyKinematics k40 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitude, height,
                oldLatitude, oldHeight);

        final BodyKinematics k41 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight);

        final BodyKinematics k42 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight);

        final BodyKinematics k43 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, heightDistance,
                oldLatitude, oldHeightDistance);

        final BodyKinematics k44 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, heightDistance,
                oldLatitude, oldHeightDistance);

        final BodyKinematics k45 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, heightDistance,
                oldLatitudeAngle, oldHeightDistance);

        final BodyKinematics k46 = estimator.estimateAndReturnNew(
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
        assertEquals(k1, k17);
        assertEquals(k1, k18);
        assertEquals(k1, k19);
        assertEquals(k1, k20);
        assertEquals(k1, k21);
        assertEquals(k1, k22);
        assertEquals(k1, k23);
        assertEquals(k1, k24);
        assertEquals(k1, k25);
        assertEquals(k1, k26);
        assertEquals(k1, k27);
        assertEquals(k1, k28);
        assertEquals(k1, k29);
        assertEquals(k1, k30);
        assertEquals(k1, k31);
        assertEquals(k1, k32);
        assertEquals(k1, k33);
        assertEquals(k1, k34);
        assertEquals(k1, k35);
        assertEquals(k1, k36);
        assertEquals(k1, k37);
        assertEquals(k1, k38);
        assertEquals(k1, k39);
        assertEquals(k1, k40);
        assertEquals(k1, k41);
        assertEquals(k1, k42);
        assertEquals(k1, k43);
        assertEquals(k1, k44);
        assertEquals(k1, k45);
        assertEquals(k1, k46);
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
            final double longitude = frame.getLongitude();
            final double height = frame.getHeight();

            final double oldLatitude = oldFrame.getLatitude();
            final double oldLongitude = oldFrame.getLongitude();
            final double oldHeight = oldFrame.getHeight();

            final BodyKinematics k1 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight,
                    k1);

            final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
            final BodyKinematics k2 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight,
                    k2);

            final NEDVelocity velocity = new NEDVelocity(vn, ve, vd);
            final NEDVelocity oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
            final BodyKinematics k3 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS,
                    c, oldC, velocity, oldVelocity, latitude, height,
                    oldLatitude, oldHeight, k3);

            final BodyKinematics k4 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, c, oldC, velocity, oldVelocity,
                    latitude, height, oldLatitude, oldHeight, k4);

            final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
            final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
            final Distance heightDistance = new Distance(height, DistanceUnit.METER);
            final Distance oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);
            final BodyKinematics k5 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity,
                    latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance,
                    k5);

            final BodyKinematics k6 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, c, oldC, velocity, oldVelocity,
                    latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance,
                    k6);

            final NEDPosition position = new NEDPosition(latitude, longitude, height);
            final NEDPosition oldPosition = new NEDPosition(oldLatitude, oldLongitude,
                    oldHeight);
            final BodyKinematics k7 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, position, oldPosition, k7);

            final BodyKinematics k8 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                    position, oldPosition, k8);

            final Speed speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
            final Speed speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
            final Speed speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);
            final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
            final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
            final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
            final BodyKinematics k9 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                    oldSpeedN, oldSpeedE, oldSpeedD, position, oldPosition, k9);

            final BodyKinematics k10 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, c, oldC, speedN, speedE, speedD,
                    oldSpeedN, oldSpeedE, oldSpeedD, position, oldPosition, k10);

            final BodyKinematics k11 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity,
                    position, oldPosition, k11);

            final BodyKinematics k12 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, c, oldC, velocity, oldVelocity,
                    position, oldPosition, k12);

            final BodyKinematics k13 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, frame, oldC, oldVn, oldVe, oldVd,
                    oldLatitude, oldHeight, k13);

            final BodyKinematics k14 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                    oldLatitude, oldHeight, k14);

            final BodyKinematics k15 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity, oldLatitude,
                    oldHeight, k15);

            final BodyKinematics k16 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, frame, oldC, oldVelocity, oldLatitude,
                    oldHeight, k16);

            final BodyKinematics k17 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity,
                    oldLatitude, oldHeight, k17);

            final BodyKinematics k18 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, frame, oldC, oldVelocity, oldLatitude,
                    oldHeight, k18);

            final BodyKinematics k19 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, frame, oldC, oldVn, oldVe, oldVd,
                    oldPosition, k19);

            final BodyKinematics k20 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                    oldPosition, k20);

            final BodyKinematics k21 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, frame, oldC, oldSpeedN,
                    oldSpeedE, oldSpeedD, oldPosition, k21);

            final BodyKinematics k22 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, frame, oldC, oldSpeedN, oldSpeedE,
                    oldSpeedD, oldPosition, k22);

            final BodyKinematics k23 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity,
                    oldPosition, k23);

            final BodyKinematics k24 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, frame, oldC, oldVelocity, oldPosition,
                    k24);

            final BodyKinematics k25 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, c, vn, ve, vd, latitude, height,
                    oldFrame, k25);

            final BodyKinematics k26 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, c, vn, ve, vd, latitude, height,
                    oldFrame, k26);

            final BodyKinematics k27 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, c, velocity, latitude, height,
                    oldFrame, k27);

            final BodyKinematics k28 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, c, velocity, latitude, height, oldFrame,
                    k28);

            final BodyKinematics k29 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, c, velocity, latitudeAngle,
                    heightDistance, oldFrame, k29);

            final BodyKinematics k30 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, c, velocity, latitudeAngle, heightDistance,
                    oldFrame, k30);

            final BodyKinematics k31 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, c, vn, ve, vd, position, oldFrame,
                    k31);

            final BodyKinematics k32 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, c, vn, ve, vd, position, oldFrame, k32);

            final BodyKinematics k33 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, c, speedN, speedE, speedD,
                    position, oldFrame, k33);

            final BodyKinematics k34 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, c, speedN, speedE, speedD, position,
                    oldFrame, k34);

            final BodyKinematics k35 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, c, velocity, position, oldFrame,
                    k35);

            final BodyKinematics k36 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, c, velocity, position, oldFrame, k36);

            final BodyKinematics k37 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, frame, oldFrame, k37);

            final BodyKinematics k38 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, frame, oldFrame, k38);

            final BodyKinematics k39 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                    oldSpeedN, oldSpeedE, oldSpeedD, latitude, height,
                    oldLatitude, oldHeight, k39);

            final BodyKinematics k40 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, c, oldC, speedN, speedE, speedD,
                    oldSpeedN, oldSpeedE, oldSpeedD, latitude, height,
                    oldLatitude, oldHeight, k40);

            final BodyKinematics k41 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight,
                    k41);

            final BodyKinematics k42 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight,
                    k42);

            final BodyKinematics k43 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, latitude, heightDistance,
                    oldLatitude, oldHeightDistance, k43);

            final BodyKinematics k44 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, latitude, heightDistance,
                    oldLatitude, oldHeightDistance, k44);

            final BodyKinematics k45 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                    oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, heightDistance,
                    oldLatitudeAngle, oldHeightDistance, k45);

            final BodyKinematics k46 = new BodyKinematics();
            NEDKinematicsEstimator.estimateKinematics(
                    timeInterval, c, oldC, speedN, speedE, speedD,
                    oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, heightDistance,
                    oldLatitudeAngle, oldHeightDistance, k46);

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
            assertEquals(k1, k17);
            assertEquals(k1, k18);
            assertEquals(k1, k19);
            assertEquals(k1, k20);
            assertEquals(k1, k21);
            assertEquals(k1, k22);
            assertEquals(k1, k23);
            assertEquals(k1, k24);
            assertEquals(k1, k25);
            assertEquals(k1, k26);
            assertEquals(k1, k27);
            assertEquals(k1, k28);
            assertEquals(k1, k29);
            assertEquals(k1, k30);
            assertEquals(k1, k31);
            assertEquals(k1, k32);
            assertEquals(k1, k33);
            assertEquals(k1, k34);
            assertEquals(k1, k35);
            assertEquals(k1, k36);
            assertEquals(k1, k37);
            assertEquals(k1, k38);
            assertEquals(k1, k39);
            assertEquals(k1, k40);
            assertEquals(k1, k41);
            assertEquals(k1, k42);
            assertEquals(k1, k43);
            assertEquals(k1, k44);
            assertEquals(k1, k45);
            assertEquals(k1, k46);

            final BodyKinematics k = estimateKinematics(TIME_INTERVAL_SECONDS,
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
            final double longitude = frame.getLongitude();
            final double height = frame.getHeight();

            final double oldLatitude = oldFrame.getLatitude();
            final double oldLongitude = oldFrame.getLongitude();
            final double oldHeight = oldFrame.getHeight();

            final BodyKinematics k1 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);

            final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
            final BodyKinematics k2 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);

            final NEDVelocity velocity = new NEDVelocity(vn, ve, vd);
            final NEDVelocity oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
            final BodyKinematics k3 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS,
                    c, oldC, velocity, oldVelocity, latitude, height,
                    oldLatitude, oldHeight);

            final BodyKinematics k4 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, c, oldC, velocity, oldVelocity,
                    latitude, height, oldLatitude, oldHeight);

            final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
            final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
            final Distance heightDistance = new Distance(height, DistanceUnit.METER);
            final Distance oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);
            final BodyKinematics k5 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity,
                    latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance);

            final BodyKinematics k6 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, c, oldC, velocity, oldVelocity,
                    latitudeAngle, heightDistance, oldLatitudeAngle, oldHeightDistance);

            final NEDPosition position = new NEDPosition(latitude, longitude, height);
            final NEDPosition oldPosition = new NEDPosition(oldLatitude, oldLongitude,
                    oldHeight);
            final BodyKinematics k7 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, position, oldPosition);

            final BodyKinematics k8 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                    position, oldPosition);

            final Speed speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
            final Speed speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
            final Speed speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);
            final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
            final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
            final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
            final BodyKinematics k9 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                    oldSpeedN, oldSpeedE, oldSpeedD, position, oldPosition);

            final BodyKinematics k10 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, c, oldC, speedN, speedE, speedD,
                    oldSpeedN, oldSpeedE, oldSpeedD, position, oldPosition);

            final BodyKinematics k11 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, c, oldC, velocity, oldVelocity,
                    position, oldPosition);

            final BodyKinematics k12 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, c, oldC, velocity, oldVelocity,
                    position, oldPosition);

            final BodyKinematics k13 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, frame, oldC, oldVn, oldVe, oldVd,
                    oldLatitude, oldHeight);

            final BodyKinematics k14 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                    oldLatitude, oldHeight);

            final BodyKinematics k15 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity, oldLatitude,
                    oldHeight);

            final BodyKinematics k16 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, frame, oldC, oldVelocity, oldLatitude,
                    oldHeight);

            final BodyKinematics k17 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity,
                    oldLatitude, oldHeight);

            final BodyKinematics k18 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, frame, oldC, oldVelocity, oldLatitude,
                    oldHeight);

            final BodyKinematics k19 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, frame, oldC, oldVn, oldVe, oldVd,
                    oldPosition);

            final BodyKinematics k20 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                    oldPosition);

            final BodyKinematics k21 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, frame, oldC, oldSpeedN,
                    oldSpeedE, oldSpeedD, oldPosition);

            final BodyKinematics k22 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, frame, oldC, oldSpeedN, oldSpeedE,
                    oldSpeedD, oldPosition);

            final BodyKinematics k23 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, frame, oldC, oldVelocity,
                    oldPosition);

            final BodyKinematics k24 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, frame, oldC, oldVelocity, oldPosition);

            final BodyKinematics k25 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, c, vn, ve, vd, latitude, height,
                    oldFrame);

            final BodyKinematics k26 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, c, vn, ve, vd, latitude, height,
                    oldFrame);

            final BodyKinematics k27 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, c, velocity, latitude, height,
                    oldFrame);

            final BodyKinematics k28 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, c, velocity, latitude, height, oldFrame);

            final BodyKinematics k29 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, c, velocity, latitudeAngle,
                    heightDistance, oldFrame);

            final BodyKinematics k30 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, c, velocity, latitudeAngle, heightDistance,
                    oldFrame);

            final BodyKinematics k31 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, c, vn, ve, vd, position, oldFrame);

            final BodyKinematics k32 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, c, vn, ve, vd, position, oldFrame);

            final BodyKinematics k33 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, c, speedN, speedE, speedD,
                    position, oldFrame);

            final BodyKinematics k34 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, c, speedN, speedE, speedD, position,
                    oldFrame);

            final BodyKinematics k35 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, c, velocity, position, oldFrame);

            final BodyKinematics k36 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, c, velocity, position, oldFrame);

            final BodyKinematics k37 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, frame, oldFrame);

            final BodyKinematics k38 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, frame, oldFrame);

            final BodyKinematics k39 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                    oldSpeedN, oldSpeedE, oldSpeedD, latitude, height,
                    oldLatitude, oldHeight);

            final BodyKinematics k40 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, c, oldC, speedN, speedE, speedD,
                    oldSpeedN, oldSpeedE, oldSpeedD, latitude, height,
                    oldLatitude, oldHeight);

            final BodyKinematics k41 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight);

            final BodyKinematics k42 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight);

            final BodyKinematics k43 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, latitude, heightDistance,
                    oldLatitude, oldHeightDistance);

            final BodyKinematics k44 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    timeInterval, c, oldC, vn, ve, vd,
                    oldVn, oldVe, oldVd, latitude, heightDistance,
                    oldLatitude, oldHeightDistance);

            final BodyKinematics k45 = NEDKinematicsEstimator
                    .estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                    oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, heightDistance,
                    oldLatitudeAngle, oldHeightDistance);

            final BodyKinematics k46 = NEDKinematicsEstimator
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
            assertEquals(k1, k17);
            assertEquals(k1, k18);
            assertEquals(k1, k19);
            assertEquals(k1, k20);
            assertEquals(k1, k21);
            assertEquals(k1, k22);
            assertEquals(k1, k23);
            assertEquals(k1, k24);
            assertEquals(k1, k25);
            assertEquals(k1, k26);
            assertEquals(k1, k27);
            assertEquals(k1, k28);
            assertEquals(k1, k29);
            assertEquals(k1, k30);
            assertEquals(k1, k31);
            assertEquals(k1, k32);
            assertEquals(k1, k33);
            assertEquals(k1, k34);
            assertEquals(k1, k35);
            assertEquals(k1, k36);
            assertEquals(k1, k37);
            assertEquals(k1, k38);
            assertEquals(k1, k39);
            assertEquals(k1, k40);
            assertEquals(k1, k41);
            assertEquals(k1, k42);
            assertEquals(k1, k43);
            assertEquals(k1, k44);
            assertEquals(k1, k45);
            assertEquals(k1, k46);

            final BodyKinematics k = estimateKinematics(TIME_INTERVAL_SECONDS,
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

        final BodyKinematics k = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(
                0.0, c, oldC, vn, ve, vd, oldVn, oldVe, oldVd,
                latitude, height, oldLatitude, oldHeight);

        assertEquals(k.getFx(), 0.0, 0.0);
        assertEquals(k.getFy(), 0.0, 0.0);
        assertEquals(k.getFz(), 0.0, 0.0);

        assertEquals(k.getAngularRateX(), 0.0, 0.0);
        assertEquals(k.getAngularRateY(), 0.0, 0.0);
        assertEquals(k.getAngularRateZ(), 0.0, 0.0);

        final BodyKinematics k2 = estimateKinematics(0.0, c, oldC,
                vn, ve, vd, oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);

        assertTrue(k2.equals(k, 0.0));
    }

    @Test
    public void testCompareKinematics() throws InvalidSourceAndDestinationFrameTypeException,
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

            final BodyKinematics nedK = NEDKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, newNedFrame, oldNedFrame);
            final BodyKinematics ecefK = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame);
            final BodyKinematics eciK = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, newEciFrame, oldEciFrame);

            final double nedSpecificForceNorm = nedK.getSpecificForceNorm();
            final double ecefSpecificForceNorm = ecefK.getSpecificForceNorm();
            final double eciSpecificForceNorm = eciK.getSpecificForceNorm();

            final double nedAngularRateNorm = nedK.getAngularRateNorm();
            final double ecefAngularRateNorm = ecefK.getAngularRateNorm();
            final double eciAngularRateNorm = eciK.getAngularRateNorm();

            assertEquals(ecefSpecificForceNorm, nedSpecificForceNorm, LARGE_ABSOLUTE_ERROR);
            assertEquals(eciSpecificForceNorm, nedSpecificForceNorm, LARGE_ABSOLUTE_ERROR);

            assertEquals(ecefAngularRateNorm, nedAngularRateNorm, LARGE_ABSOLUTE_ERROR);
            assertEquals(eciAngularRateNorm, nedAngularRateNorm, LARGE_ABSOLUTE_ERROR);

            assertEquals(ecefK.getFx(), nedK.getFx(), LARGE_ABSOLUTE_ERROR);
            assertEquals(ecefK.getFy(), nedK.getFy(), LARGE_ABSOLUTE_ERROR);
            assertEquals(ecefK.getFz(), nedK.getFz(), LARGE_ABSOLUTE_ERROR);
            assertEquals(ecefK.getAngularRateX(), nedK.getAngularRateX(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(ecefK.getAngularRateY(), nedK.getAngularRateY(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(ecefK.getAngularRateZ(), nedK.getAngularRateZ(),
                    LARGE_ABSOLUTE_ERROR);

            assertEquals(eciK.getFx(), nedK.getFx(), LARGE_ABSOLUTE_ERROR);
            assertEquals(eciK.getFy(), nedK.getFy(), LARGE_ABSOLUTE_ERROR);
            assertEquals(eciK.getFz(), nedK.getFz(), LARGE_ABSOLUTE_ERROR);
            assertEquals(eciK.getAngularRateX(), nedK.getAngularRateX(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(eciK.getAngularRateY(), nedK.getAngularRateY(),
                    LARGE_ABSOLUTE_ERROR);
            assertEquals(eciK.getAngularRateZ(), nedK.getAngularRateZ(),
                    LARGE_ABSOLUTE_ERROR);

            assertTrue(ecefK.equals(nedK, LARGE_ABSOLUTE_ERROR));
            assertTrue(eciK.equals(nedK, LARGE_ABSOLUTE_ERROR));
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

    private static BodyKinematics estimateKinematics(final double timeInterval,
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

            return new BodyKinematics(fx, fy, fz,
                    angularRateX, angularRateY, angularRateZ);

        } else {
            // If time interval is zero, set angular rate and specific force to zero
            return new BodyKinematics();
        }
    }
}
