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

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.inertial.ECEFKinematics;
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

    private static final double ABSOLUTE_ERROR = 1e-4;

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

        final ECEFKinematics k1 = new ECEFKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight,
                k1);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final ECEFKinematics k2 = new ECEFKinematics();
        estimator.estimate(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight,
                k2);

        final ECEFKinematics k3 = new ECEFKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight, k3);

        final ECEFKinematics k4 = new ECEFKinematics();
        estimator.estimate(timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight, k4);

        final ECEFKinematics k5 = new ECEFKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, vn, ve, vd, latitude, height,
                oldFrame, k5);

        final ECEFKinematics k6 = new ECEFKinematics();
        estimator.estimate(timeInterval, c, vn, ve, vd, latitude, height,
                oldFrame, k6);

        final ECEFKinematics k7 = new ECEFKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, frame, oldFrame, k7);

        final ECEFKinematics k8 = new ECEFKinematics();
        estimator.estimate(timeInterval, frame, oldFrame, k8);

        final Speed speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final Speed speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final Speed speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);

        final ECEFKinematics k9 = new ECEFKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitude, height,
                oldLatitude, oldHeight, k9);

        final ECEFKinematics k10 = new ECEFKinematics();
        estimator.estimate(timeInterval, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitude, height,
                oldLatitude, oldHeight, k10);

        final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);

        final ECEFKinematics k11 = new ECEFKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight,
                k11);

        final ECEFKinematics k12 = new ECEFKinematics();
        estimator.estimate(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight,
                k12);

        final Distance heightDistance = new Distance(height, DistanceUnit.METER);
        final Distance oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);

        final ECEFKinematics k13 = new ECEFKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, heightDistance,
                oldLatitude, oldHeightDistance, k13);

        final ECEFKinematics k14 = new ECEFKinematics();
        estimator.estimate(timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, heightDistance,
                oldLatitude, oldHeightDistance, k14);

        final ECEFKinematics k15 = new ECEFKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, heightDistance,
                oldLatitudeAngle, oldHeightDistance, k15);

        final ECEFKinematics k16 = new ECEFKinematics();
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

        final ECEFKinematics k1 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final ECEFKinematics k2 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, height, oldLatitude, oldHeight);

        final ECEFKinematics k3 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight);

        final ECEFKinematics k4 = estimator.estimateAndReturnNew(
                timeInterval, frame, oldC, oldVn, oldVe, oldVd,
                oldLatitude, oldHeight);

        final ECEFKinematics k5 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, vn, ve, vd, latitude, height,
                oldFrame);

        final ECEFKinematics k6 = estimator.estimateAndReturnNew(
                timeInterval, c, vn, ve, vd, latitude, height,
                oldFrame);

        final ECEFKinematics k7 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, frame, oldFrame);

        final ECEFKinematics k8 = estimator.estimateAndReturnNew(
                timeInterval, frame, oldFrame);

        final Speed speedN = new Speed(vn, SpeedUnit.METERS_PER_SECOND);
        final Speed speedE = new Speed(ve, SpeedUnit.METERS_PER_SECOND);
        final Speed speedD = new Speed(vd, SpeedUnit.METERS_PER_SECOND);

        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);

        final ECEFKinematics k9 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitude, height,
                oldLatitude, oldHeight);

        final ECEFKinematics k10 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitude, height,
                oldLatitude, oldHeight);

        final Angle latitudeAngle = new Angle(latitude, AngleUnit.RADIANS);
        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);

        final ECEFKinematics k11 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight);

        final ECEFKinematics k12 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitudeAngle, height, oldLatitudeAngle, oldHeight);

        final Distance heightDistance = new Distance(height, DistanceUnit.METER);
        final Distance oldHeightDistance = new Distance(oldHeight, DistanceUnit.METER);

        final ECEFKinematics k13 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, heightDistance,
                oldLatitude, oldHeightDistance);

        final ECEFKinematics k14 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC, vn, ve, vd,
                oldVn, oldVe, oldVd, latitude, heightDistance,
                oldLatitude, oldHeightDistance);

        final ECEFKinematics k15 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC, speedN, speedE, speedD,
                oldSpeedN, oldSpeedE, oldSpeedD, latitudeAngle, heightDistance,
                oldLatitudeAngle, oldHeightDistance);

        final ECEFKinematics k16 = estimator.estimateAndReturnNew(
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
}
