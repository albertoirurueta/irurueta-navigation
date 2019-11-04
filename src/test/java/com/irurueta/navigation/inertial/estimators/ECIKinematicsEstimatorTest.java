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
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.ECIFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoECIFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.ECIGravitation;
import com.irurueta.navigation.inertial.ECIKinematics;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class ECIKinematicsEstimatorTest {

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

    private static final double ABSOLUTE_ERROR = 1e-5;

    private static final int TIMES = 100;

    @Test
    public void testEstimate() throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(oldNedFrame);
        final ECIFrame oldEciFrame = ECEFtoECIFrameConverter
                .convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(newNedFrame);
        final ECIFrame newEciFrame = ECEFtoECIFrameConverter
                .convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame);

        final CoordinateTransformation c = newEciFrame.getCoordinateTransformation();
        final CoordinateTransformation oldC = oldEciFrame.getCoordinateTransformation();

        final double vx = newEciFrame.getVx();
        final double vy = newEciFrame.getVy();
        final double vz = newEciFrame.getVz();

        final double oldVx = oldEciFrame.getVx();
        final double oldVy = oldEciFrame.getVy();
        final double oldVz = oldEciFrame.getVz();

        final double x = newEciFrame.getX();
        final double y = newEciFrame.getY();
        final double z = newEciFrame.getZ();

        final ECIKinematicsEstimator estimator = new ECIKinematicsEstimator();

        final ECIKinematics k1 = new ECIKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, k1);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final ECIKinematics k2 = new ECIKinematics();
        estimator.estimate(timeInterval, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, k2);

        final ECIKinematics k3 = new ECIKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, newEciFrame,
                oldC, oldVx, oldVy, oldVz, k3);

        final ECIKinematics k4 = new ECIKinematics();
        estimator.estimate(timeInterval, newEciFrame,
                oldC, oldVx, oldVy, oldVz, k4);

        final ECIKinematics k5 = new ECIKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, newEciFrame,
                oldEciFrame, k5);

        final ECIKinematics k6 = new ECIKinematics();
        estimator.estimate(timeInterval, newEciFrame,
                oldEciFrame, k6);

        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        final ECIKinematics k7 = new ECIKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z, k7);

        final ECIKinematics k8 = new ECIKinematics();
        estimator.estimate(timeInterval, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z, k8);

        final ECIKinematics k9 = new ECIKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, newEciFrame,
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ, k9);

        final ECIKinematics k10 = new ECIKinematics();
        estimator.estimate(timeInterval, newEciFrame,
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ, k10);

        final Point3D position = new InhomogeneousPoint3D(x, y, z);
        final ECIKinematics k11 = new ECIKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, position, k11);

        final ECIKinematics k12 = new ECIKinematics();
        estimator.estimate(timeInterval, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, position, k12);

        final ECIKinematics k13 = new ECIKinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position, k13);

        final ECIKinematics k14 = new ECIKinematics();
        estimator.estimate(timeInterval, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position, k14);

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
    }

    @Test
    public void testEstimateAndReturnNew()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(oldNedFrame);
        final ECIFrame oldEciFrame = ECEFtoECIFrameConverter
                .convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(newNedFrame);
        final ECIFrame newEciFrame = ECEFtoECIFrameConverter
                .convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame);

        final CoordinateTransformation c = newEciFrame.getCoordinateTransformation();
        final CoordinateTransformation oldC = oldEciFrame.getCoordinateTransformation();

        final double vx = newEciFrame.getVx();
        final double vy = newEciFrame.getVy();
        final double vz = newEciFrame.getVz();

        final double oldVx = oldEciFrame.getVx();
        final double oldVy = oldEciFrame.getVy();
        final double oldVz = oldEciFrame.getVz();

        final double x = newEciFrame.getX();
        final double y = newEciFrame.getY();
        final double z = newEciFrame.getZ();

        final ECIKinematicsEstimator estimator = new ECIKinematicsEstimator();

        final ECIKinematics k1 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final ECIKinematics k2 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        final ECIKinematics k3 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, newEciFrame,
                oldC, oldVx, oldVy, oldVz);

        final ECIKinematics k4 = estimator.estimateAndReturnNew(
                timeInterval, newEciFrame,
                oldC, oldVx, oldVy, oldVz);

        final ECIKinematics k5 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, newEciFrame,
                oldEciFrame);

        final ECIKinematics k6 = estimator.estimateAndReturnNew(
                timeInterval, newEciFrame,
                oldEciFrame);

        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        final ECIKinematics k7 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z);

        final ECIKinematics k8 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z);

        final ECIKinematics k9 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, newEciFrame,
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ);

        final ECIKinematics k10 = estimator.estimateAndReturnNew(
                timeInterval, newEciFrame,
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ);

        final Point3D position = new InhomogeneousPoint3D(x, y, z);
        final ECIKinematics k11 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, position);

        final ECIKinematics k12 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, position);

        final ECIKinematics k13 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position);

        final ECIKinematics k14 = estimator.estimateAndReturnNew(
                timeInterval, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position);

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
    }

    @Test
    public void testEstimateKinematics()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, WrongSizeException,
            RankDeficientMatrixException, DecomposerException {

        for (int t = 0; t < TIMES; t++) {
            final NEDFrame oldNedFrame = createOldNedFrame();
            final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(oldNedFrame);
            final ECIFrame oldEciFrame = ECEFtoECIFrameConverter
                    .convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefFrame);

            final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
            final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(newNedFrame);
            final ECIFrame newEciFrame = ECEFtoECIFrameConverter
                    .convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame);

            final CoordinateTransformation c = newEciFrame.getCoordinateTransformation();
            final CoordinateTransformation oldC = oldEciFrame.getCoordinateTransformation();

            final double vx = newEciFrame.getVx();
            final double vy = newEciFrame.getVy();
            final double vz = newEciFrame.getVz();

            final double oldVx = oldEciFrame.getVx();
            final double oldVy = oldEciFrame.getVy();
            final double oldVz = oldEciFrame.getVz();

            final double x = newEciFrame.getX();
            final double y = newEciFrame.getY();
            final double z = newEciFrame.getZ();

            final ECIKinematics k1 = new ECIKinematics();
            ECIKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC,
                    vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, k1);

            final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
            final ECIKinematics k2 = new ECIKinematics();
            ECIKinematicsEstimator.estimateKinematics(timeInterval, c, oldC,
                    vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, k2);

            final ECIKinematics k3 = new ECIKinematics();
            ECIKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEciFrame,
                    oldC, oldVx, oldVy, oldVz, k3);

            final ECIKinematics k4 = new ECIKinematics();
            ECIKinematicsEstimator.estimateKinematics(timeInterval, newEciFrame,
                    oldC, oldVx, oldVy, oldVz, k4);

            final ECIKinematics k5 = new ECIKinematics();
            ECIKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEciFrame,
                    oldEciFrame, k5);

            final ECIKinematics k6 = new ECIKinematics();
            ECIKinematicsEstimator.estimateKinematics(timeInterval, newEciFrame,
                    oldEciFrame, k6);

            final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
            final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
            final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

            final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
            final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
            final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

            final ECIKinematics k7 = new ECIKinematics();
            ECIKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC,
                    speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z, k7);

            final ECIKinematics k8 = new ECIKinematics();
            ECIKinematicsEstimator.estimateKinematics(timeInterval, c, oldC,
                    speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z, k8);

            final ECIKinematics k9 = new ECIKinematics();
            ECIKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEciFrame,
                    oldC, oldSpeedX, oldSpeedY, oldSpeedZ, k9);

            final ECIKinematics k10 = new ECIKinematics();
            ECIKinematicsEstimator.estimateKinematics(timeInterval, newEciFrame,
                    oldC, oldSpeedX, oldSpeedY, oldSpeedZ, k10);

            final Point3D position = new InhomogeneousPoint3D(x, y, z);
            final ECIKinematics k11 = new ECIKinematics();
            ECIKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC,
                    vx, vy, vz, oldVx, oldVy, oldVz, position, k11);

            final ECIKinematics k12 = new ECIKinematics();
            ECIKinematicsEstimator.estimateKinematics(timeInterval, c, oldC,
                    vx, vy, vz, oldVx, oldVy, oldVz, position, k12);

            final ECIKinematics k13 = new ECIKinematics();
            ECIKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC,
                    speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position, k13);

            final ECIKinematics k14 = new ECIKinematics();
            ECIKinematicsEstimator.estimateKinematics(timeInterval, c, oldC,
                    speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position, k14);

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

            final ECIKinematics k = estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC,
                    vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

            assertTrue(k1.equals(k, ABSOLUTE_ERROR));
        }
    }

    @Test
    public void testEstimateKinematicsAndReturnNew()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, WrongSizeException,
            RankDeficientMatrixException, DecomposerException {

        for (int t = 0; t < TIMES; t++) {
            final NEDFrame oldNedFrame = createOldNedFrame();
            final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(oldNedFrame);
            final ECIFrame oldEciFrame = ECEFtoECIFrameConverter
                    .convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefFrame);

            final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
            final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(newNedFrame);
            final ECIFrame newEciFrame = ECEFtoECIFrameConverter
                    .convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame);

            final CoordinateTransformation c = newEciFrame.getCoordinateTransformation();
            final CoordinateTransformation oldC = oldEciFrame.getCoordinateTransformation();

            final double vx = newEciFrame.getVx();
            final double vy = newEciFrame.getVy();
            final double vz = newEciFrame.getVz();

            final double oldVx = oldEciFrame.getVx();
            final double oldVy = oldEciFrame.getVy();
            final double oldVz = oldEciFrame.getVz();

            final double x = newEciFrame.getX();
            final double y = newEciFrame.getY();
            final double z = newEciFrame.getZ();

            final ECIKinematics k1 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, c, oldC,
                    vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

            final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
            final ECIKinematics k2 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(
                    timeInterval, c, oldC,
                    vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

            final ECIKinematics k3 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, newEciFrame,
                    oldC, oldVx, oldVy, oldVz);

            final ECIKinematics k4 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(
                    timeInterval, newEciFrame,
                    oldC, oldVx, oldVy, oldVz);

            final ECIKinematics k5 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, newEciFrame,
                    oldEciFrame);

            final ECIKinematics k6 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(
                    timeInterval, newEciFrame,
                    oldEciFrame);

            final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
            final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
            final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

            final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
            final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
            final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

            final ECIKinematics k7 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, c, oldC,
                    speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z);

            final ECIKinematics k8 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(
                    timeInterval, c, oldC,
                    speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z);

            final ECIKinematics k9 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, newEciFrame,
                    oldC, oldSpeedX, oldSpeedY, oldSpeedZ);

            final ECIKinematics k10 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(
                    timeInterval, newEciFrame,
                    oldC, oldSpeedX, oldSpeedY, oldSpeedZ);

            final Point3D position = new InhomogeneousPoint3D(x, y, z);
            final ECIKinematics k11 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, c, oldC,
                    vx, vy, vz, oldVx, oldVy, oldVz, position);

            final ECIKinematics k12 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(
                    timeInterval, c, oldC,
                    vx, vy, vz, oldVx, oldVy, oldVz, position);

            final ECIKinematics k13 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(
                    TIME_INTERVAL_SECONDS, c, oldC,
                    speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position);

            final ECIKinematics k14 = ECIKinematicsEstimator.estimateKinematicsAndReturnNew(
                    timeInterval, c, oldC,
                    speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position);

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

            final ECIKinematics k = estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC,
                    vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

            assertTrue(k1.equals(k, ABSOLUTE_ERROR));
        }
    }

    @Test(expected = IllegalArgumentException.class)
    public void testEstimateKinematicsWhenNegativeIntervalThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(oldNedFrame);
        final ECIFrame oldEciFrame = ECEFtoECIFrameConverter
                .convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(newNedFrame);
        final ECIFrame newEciFrame = ECEFtoECIFrameConverter
                .convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame);

        final CoordinateTransformation c = newEciFrame.getCoordinateTransformation();
        final CoordinateTransformation oldC = oldEciFrame.getCoordinateTransformation();

        final double vx = newEciFrame.getVx();
        final double vy = newEciFrame.getVy();
        final double vz = newEciFrame.getVz();

        final double oldVx = oldEciFrame.getVx();
        final double oldVy = oldEciFrame.getVy();
        final double oldVz = oldEciFrame.getVz();

        final double x = newEciFrame.getX();
        final double y = newEciFrame.getY();
        final double z = newEciFrame.getZ();

        ECIKinematicsEstimator.estimateKinematicsAndReturnNew(-1.0, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);
    }

    @Test(expected = IllegalArgumentException.class)
    public void testEstimateKinematicsWhenInvalidCoordinateTransformationThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(oldNedFrame);
        final ECIFrame oldEciFrame = ECEFtoECIFrameConverter
                .convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(newNedFrame);
        final ECIFrame newEciFrame = ECEFtoECIFrameConverter
                .convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame);

        final CoordinateTransformation c = newEciFrame.getCoordinateTransformation();
        final CoordinateTransformation oldC = oldEciFrame.getCoordinateTransformation();

        final double vx = newEciFrame.getVx();
        final double vy = newEciFrame.getVy();
        final double vz = newEciFrame.getVz();

        final double oldVx = oldEciFrame.getVx();
        final double oldVy = oldEciFrame.getVy();
        final double oldVz = oldEciFrame.getVz();

        final double x = newEciFrame.getX();
        final double y = newEciFrame.getY();
        final double z = newEciFrame.getZ();

        c.setDestinationType(FrameType.BODY_FRAME);
        ECIKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);
    }

    @Test(expected = IllegalArgumentException.class)
    public void testEstimateKinematicsWhenInvalidOldCoordinateTransformationThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(oldNedFrame);
        final ECIFrame oldEciFrame = ECEFtoECIFrameConverter
                .convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(newNedFrame);
        final ECIFrame newEciFrame = ECEFtoECIFrameConverter
                .convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame);

        final CoordinateTransformation c = newEciFrame.getCoordinateTransformation();
        final CoordinateTransformation oldC = oldEciFrame.getCoordinateTransformation();

        final double vx = newEciFrame.getVx();
        final double vy = newEciFrame.getVy();
        final double vz = newEciFrame.getVz();

        final double oldVx = oldEciFrame.getVx();
        final double oldVy = oldEciFrame.getVy();
        final double oldVz = oldEciFrame.getVz();

        final double x = newEciFrame.getX();
        final double y = newEciFrame.getY();
        final double z = newEciFrame.getZ();

        oldC.setDestinationType(FrameType.BODY_FRAME);
        ECIKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);
    }

    @Test
    public void testEstimateKinematicsWhenZeroTimeIntervalReturnsZeroValues()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(oldNedFrame);
        final ECIFrame oldEciFrame = ECEFtoECIFrameConverter
                .convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(newNedFrame);
        final ECIFrame newEciFrame = ECEFtoECIFrameConverter
                .convertECEFtoECIAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame);

        final CoordinateTransformation c = newEciFrame.getCoordinateTransformation();
        final CoordinateTransformation oldC = oldEciFrame.getCoordinateTransformation();

        final double vx = newEciFrame.getVx();
        final double vy = newEciFrame.getVy();
        final double vz = newEciFrame.getVz();

        final double oldVx = oldEciFrame.getVx();
        final double oldVy = oldEciFrame.getVy();
        final double oldVz = oldEciFrame.getVz();

        final double x = newEciFrame.getX();
        final double y = newEciFrame.getY();
        final double z = newEciFrame.getZ();

        final ECIKinematics k = ECIKinematicsEstimator
                .estimateKinematicsAndReturnNew(0.0, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        assertEquals(k.getFx(), 0.0, 0.0);
        assertEquals(k.getFy(), 0.0, 0.0);
        assertEquals(k.getFz(), 0.0, 0.0);

        assertEquals(k.getAngularRateX(), 0.0, 0.0);
        assertEquals(k.getAngularRateY(), 0.0, 0.0);
        assertEquals(k.getAngularRateZ(), 0.0, 0.0);
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

    private ECIKinematics estimateKinematics(final double timeInterval,
                                             final CoordinateTransformation c,
                                             final CoordinateTransformation oldC,
                                             final double vx, final double vy, final double vz,
                                             final double oldVx, final double oldVy, final double oldVz,
                                             final double x, final double y, final double z)
            throws WrongSizeException, RankDeficientMatrixException, DecomposerException {

        if (timeInterval > 0.0) {
            // Obtain coordinate transformation matrix from the old attitude to the new
            final Matrix cBi = c.getMatrix();
            final Matrix oldCbi = oldC.getMatrix();
            final Matrix cOldNew = cBi.transposeAndReturnNew().multiplyAndReturnNew(oldCbi);

            // Calculate the approximate angular rate
            final Matrix alphaIbb = new Matrix(3, 1);
            alphaIbb.setElementAtIndex(0, 0.5 * (cOldNew.getElementAt(1, 2)
                    - cOldNew.getElementAt(2, 1)));
            alphaIbb.setElementAtIndex(1, 0.5 * (cOldNew.getElementAt(2, 0)
                    - cOldNew.getElementAt(0, 2)));
            alphaIbb.setElementAtIndex(2, 0.5 * (cOldNew.getElementAt(0, 1)
                    - cOldNew.getElementAt(1, 0)));

            // Calculate and apply the scaling factor
            final double temp = Math.acos(0.5 * (Utils.trace(cOldNew) - 1.0));
            if (temp > SCALING_THRESHOLD) {
                // scaling is 1 if temp is less than this
                alphaIbb.multiplyByScalar(temp / Math.sin(temp));
            }

            // Calculate the angular rate
            final Matrix omegaIbb = alphaIbb.multiplyByScalarAndReturnNew(1.0 / timeInterval);

            // Calculate the specific force resolved about ECI-frame axes
            // From (5.18) and (5.20)
            final Matrix vIbi = new Matrix(3, 1);
            vIbi.setElementAtIndex(0, vx);
            vIbi.setElementAtIndex(1, vy);
            vIbi.setElementAtIndex(2, vz);

            final Matrix oldVibi = new Matrix(3, 1);
            oldVibi.setElementAtIndex(0, oldVx);
            oldVibi.setElementAtIndex(1, oldVy);
            oldVibi.setElementAtIndex(2, oldVz);

            final ECIGravitation gravitation = ECIGravitationEstimator.estimateGravitationAndReturnNew(x, y, z);
            final Matrix g = gravitation.asMatrix();

            final Matrix fIbi = vIbi.subtractAndReturnNew(oldVibi)
                    .multiplyByScalarAndReturnNew(1.0 / timeInterval).subtractAndReturnNew(g);

            // Calculate the average body-to-ECI-frame coordinate transformation
            // matrix over the update interval using (5.84)
            final double magAlpha = Utils.normF(alphaIbb);
            final Matrix skewAlpha = Utils.skewMatrix(alphaIbb);
            final Matrix aveCbi;
            if (magAlpha > ALPHA_THRESHOLD) {
                final double magAlpha2 = Math.pow(magAlpha, 2.0);
                aveCbi = oldCbi.multiplyAndReturnNew(Matrix.identity(3, 3)
                        .addAndReturnNew(skewAlpha.multiplyByScalarAndReturnNew(
                                (1.0 - Math.cos(magAlpha)) / magAlpha2)).addAndReturnNew(
                                        skewAlpha.multiplyAndReturnNew(skewAlpha)
                                                .multiplyByScalarAndReturnNew(
                                                        (1.0 - Math.sin(magAlpha) / magAlpha) / magAlpha2)));
            } else {
                aveCbi = oldCbi;
            }

            // Transform specific force to body-frame resolving axes using (5.81)
            final Matrix fIbb = Utils.inverse(aveCbi).multiplyAndReturnNew(fIbi);

            final double fx = fIbb.getElementAtIndex(0);
            final double fy = fIbb.getElementAtIndex(1);
            final double fz = fIbb.getElementAtIndex(2);

            final double angularRateX = omegaIbb.getElementAtIndex(0);
            final double angularRateY = omegaIbb.getElementAtIndex(1);
            final double angularRateZ = omegaIbb.getElementAtIndex(2);

            return new ECIKinematics(fx, fy, fz, angularRateX, angularRateY, angularRateZ);
        } else {
            // If time interval is zero, set angular rate and specific force to zer
            return new ECIKinematics();
        }
    }
}
