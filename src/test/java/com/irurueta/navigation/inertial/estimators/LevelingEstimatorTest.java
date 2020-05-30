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
package com.irurueta.navigation.inertial.estimators;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.NEDGravity;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.AngularSpeed;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class LevelingEstimatorTest {

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_HEIGHT = -100.0;
    private static final double MAX_HEIGHT = 100.0;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double TIME_INTERVAL = 0.02;

    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;
    private static final double ABSOLUTE_ERROR = 1e-6;

    @Test
    public void testCheckKinematicValues()
            throws WrongSizeException {

        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll, pitch, yaw, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final double fx1 = kinematics.getFx();
        final double fy1 = kinematics.getFy();
        final double fz1 = kinematics.getFz();

        // check that kinematics values are correct

        // obtain NED to body conversion matrix
        final Matrix cnb = bodyC.getMatrix();

        final NEDGravity nedGravity = NEDGravityEstimator
                .estimateGravityAndReturnNew(latitude, height);
        final Matrix g = Matrix.newFromArray(nedGravity.asArray());
        g.multiplyByScalar(-1.0);
        final Matrix f2 = cnb.multiplyAndReturnNew(g);

        final double fx2 = f2.getElementAtIndex(0);
        final double fy2 = f2.getElementAtIndex(1);
        final double fz2 = f2.getElementAtIndex(2);

        final Matrix f3 = Matrix.newFromArray(new double[]{
                Math.sin(pitch) * nedGravity.getGd(),
                -Math.cos(pitch) * Math.sin(roll) * nedGravity.getGd(),
                -Math.cos(pitch) * Math.cos(roll) * nedGravity.getGd()
        });

        final double fx3 = f3.getElementAtIndex(0);
        final double fy3 = f3.getElementAtIndex(1);
        final double fz3 = f3.getElementAtIndex(2);

        assertEquals(fx1, fx2, LARGE_ABSOLUTE_ERROR);
        assertEquals(fx2, fx3, ABSOLUTE_ERROR);

        assertEquals(fy1, fy2, LARGE_ABSOLUTE_ERROR);
        assertEquals(fy2, fy3, ABSOLUTE_ERROR);

        assertEquals(fz1, fz2, LARGE_ABSOLUTE_ERROR);
        assertEquals(fz2, fz3, ABSOLUTE_ERROR);
    }

    @Test
    public void testGetRollWhenNoRotationTheoretical() throws WrongSizeException {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = 0.0;
        final double pitch1 = 0.0;
        final double yaw1 = 0.0;

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);

        // obtain specific force neglecting north component of gravity
        final Matrix cnb = bodyC.getMatrix();
        final NEDGravity nedGravity = NEDGravityEstimator
                .estimateGravityAndReturnNew(latitude, height);
        final Matrix g = Matrix.newFromArray(nedGravity.asArray());
        g.multiplyByScalar(-1.0);
        final Matrix f = cnb.multiplyAndReturnNew(g);

        final double fy = f.getElementAtIndex(1);
        final double fz = f.getElementAtIndex(2);

        final double roll2 = LevelingEstimator.getRoll(fy, fz);

        // Because we have only taken gravity term in measured specific
        // force, the amount of error is smaller, however, in real
        // environments, measured specific force will contain a term due
        // to Earth rotation, and additional terms due to different kinds
        // of errors (bias, scaling, noise, etc).
        assertEquals(roll1, roll2, ABSOLUTE_ERROR);
    }

    @Test
    public void testGetRollWhenNoRotation() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = 0.0;
        final double pitch1 = 0.0;
        final double yaw1 = 0.0;

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final double roll2 = LevelingEstimator.getRoll(kinematics.getFy(),
                kinematics.getFz());

        // Kinematics estimator, also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term, for that reason amount of error is slightly larger
        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    public void testGetRollWithRotationTheoretical() throws WrongSizeException {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);

        // obtain specific force neglecting north component of gravity
        final Matrix cnb = bodyC.getMatrix();
        final NEDGravity nedGravity = NEDGravityEstimator
                .estimateGravityAndReturnNew(latitude, height);
        final Matrix g = Matrix.newFromArray(nedGravity.asArray());
        g.multiplyByScalar(-1.0);
        final Matrix f = cnb.multiplyAndReturnNew(g);

        final double fy = f.getElementAtIndex(1);
        final double fz = f.getElementAtIndex(2);

        final double roll2 = LevelingEstimator.getRoll(fy, fz);

        // Because we have only taken gravity term in measured specific
        // force, the amount of error is smaller, however, in real
        // environments, measured specific force will contain a term due
        // to Earth rotation, and additional terms due to different kinds
        // of errors (bias, scaling, noise, etc).
        assertEquals(roll1, roll2, ABSOLUTE_ERROR);
    }

    @Test
    public void testGetRollWithRotation() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final double roll2 = LevelingEstimator.getRoll(kinematics.getFy(),
                kinematics.getFz());

        // Kinematics estimator, also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term, for that reason amount of error is slightly larger
        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    public void testGetPitchWhenNoRotationTheoretical()
            throws WrongSizeException {

        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = 0.0;
        final double pitch1 = 0.0;
        final double yaw1 = 0.0;

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);

        // obtain specific force neglecting north component of gravity
        final Matrix cnb = bodyC.getMatrix();
        final NEDGravity nedGravity = NEDGravityEstimator
                .estimateGravityAndReturnNew(latitude, height);
        final Matrix g = Matrix.newFromArray(nedGravity.asArray());
        g.multiplyByScalar(-1.0);
        final Matrix f = cnb.multiplyAndReturnNew(g);

        final double fx = f.getElementAtIndex(0);
        final double fy = f.getElementAtIndex(1);
        final double fz = f.getElementAtIndex(2);

        final double pitch2 = LevelingEstimator.getPitch(fx, fy, fz);

        // Because we have only taken gravity term in measured specific
        // force, the amount of error is smaller, however, in real
        // environments, measured specific force will contain a term due
        // to Earth rotation, and additional terms due to different kinds
        // of errors (bias, scaling, noise, etc).
        assertEquals(pitch1, pitch2, ABSOLUTE_ERROR);
    }

    @Test
    public void testGetPitchWhenNoRotation() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = 0.0;
        final double pitch1 = 0.0;
        final double yaw1 = 0.0;

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final double pitch2 = LevelingEstimator.getPitch(
                kinematics.getFx(), kinematics.getFy(),
                kinematics.getFz());

        // Kinematics estimator, also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term, for that reason amount of error is slightly larger
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    public void testGetPitchWithRotationTheoretical() throws WrongSizeException {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);

        // obtain specific force neglecting north component of gravity
        final Matrix cnb = bodyC.getMatrix();
        final NEDGravity nedGravity = NEDGravityEstimator
                .estimateGravityAndReturnNew(latitude, height);
        final Matrix g = Matrix.newFromArray(nedGravity.asArray());
        g.multiplyByScalar(-1.0);
        final Matrix f = cnb.multiplyAndReturnNew(g);

        final double fx = f.getElementAtIndex(0);
        final double fy = f.getElementAtIndex(1);
        final double fz = f.getElementAtIndex(2);

        final double pitch2 = LevelingEstimator.getPitch(fx, fy, fz);

        // Because we have only taken gravity term in measured specific
        // force, the amount of error is smaller, however, in real
        // environments, measured specific force will contain a term due
        // to Earth rotation, and additional terms due to different kinds
        // of errors (bias, scaling, noise, etc).
        assertEquals(pitch1, pitch2, ABSOLUTE_ERROR);
    }

    @Test
    public void testGetPitchWithRotation() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final double pitch2 = LevelingEstimator.getPitch(
                kinematics.getFx(), kinematics.getFy(),
                kinematics.getFz());

        // Kinematics estimator, also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term, for that reason amount of error is slightly larger
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    public void testGetYawWhenNoRotation() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = 0.0;
        final double pitch1 = 0.0;
        final double yaw1 = 0.0;

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final double roll2 = LevelingEstimator.getRoll(kinematics.getFy(),
                kinematics.getFz());
        final double pitch2 = LevelingEstimator.getPitch(
                kinematics.getFx(), kinematics.getFy(),
                kinematics.getFz());
        final double yaw2 = LevelingEstimator.getYaw(roll2, pitch2,
                kinematics.getAngularRateX(), kinematics.getAngularRateY(),
                kinematics.getAngularRateZ());

        // Kinematics estimator, also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term, for that reason amount of error is slightly larger
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    public void testGetYawWithRotation() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final double roll2 = LevelingEstimator.getRoll(kinematics.getFy(),
                kinematics.getFz());
        final double pitch2 = LevelingEstimator.getPitch(
                kinematics.getFx(), kinematics.getFy(),
                kinematics.getFz());
        final double yaw2 = LevelingEstimator.getYaw(roll2, pitch2,
                kinematics.getAngularRateX(), kinematics.getAngularRateY(),
                kinematics.getAngularRateZ());

        // Kinematics estimator, also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term, for that reason amount of error is slightly larger
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    public void testGetYaw2() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final double yaw2 = LevelingEstimator.getYaw(
                kinematics.getFx(), kinematics.getFy(),
                kinematics.getFz(), kinematics.getAngularRateX(),
                kinematics.getAngularRateY(),
                kinematics.getAngularRateZ());

        // Kinematics estimator, also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term, for that reason amount of error is slightly larger
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    public void testGetRoll2() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final double roll2 = LevelingEstimator.getRoll(kinematics);

        // Kinematics estimator, also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term, for that reason amount of error is slightly larger
        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    public void testGetPitch2() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final double pitch2 = LevelingEstimator.getPitch(kinematics);

        // Kinematics estimator, also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term, for that reason amount of error is slightly larger
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    public void testGetYaw3() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final double yaw2 = LevelingEstimator.getYaw(kinematics);

        // Kinematics estimator, also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term, for that reason amount of error is slightly larger
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    public void testGetAttitude1() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final CoordinateTransformation bodyC2 =
                new CoordinateTransformation(FrameType.BODY_FRAME,
                        FrameType.BODY_FRAME);

        LevelingEstimator.getAttitude(
                kinematics.getFx(), kinematics.getFy(),
                kinematics.getFz(), kinematics.getAngularRateX(),
                kinematics.getAngularRateY(),
                kinematics.getAngularRateZ(), bodyC2);

        assertTrue(bodyC.equals(bodyC2, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testGetAttitude2() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final CoordinateTransformation bodyC2 =
                new CoordinateTransformation(FrameType.BODY_FRAME,
                        FrameType.BODY_FRAME);

        LevelingEstimator.getAttitude(kinematics, bodyC2);

        assertTrue(bodyC.equals(bodyC2, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testGetAttitude3() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final CoordinateTransformation bodyC2 =
                LevelingEstimator.getAttitude(
                        kinematics.getFx(),
                        kinematics.getFy(),
                        kinematics.getFz(),
                        kinematics.getAngularRateX(),
                        kinematics.getAngularRateY(),
                        kinematics.getAngularRateZ());

        assertTrue(bodyC.equals(bodyC2, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testGetAttitude4() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final CoordinateTransformation bodyC2 =
                LevelingEstimator.getAttitude(kinematics);

        assertTrue(bodyC.equals(bodyC2, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testGetRoll3() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final Acceleration fy = kinematics.getSpecificForceY();
        final Acceleration fz = kinematics.getSpecificForceZ();

        final double roll2 = LevelingEstimator.getRoll(fy, fz);

        // Kinematics estimator, also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term, for that reason amount of error is slightly larger
        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    public void testGetPitch3() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final Acceleration fx = kinematics.getSpecificForceX();
        final Acceleration fy = kinematics.getSpecificForceY();
        final Acceleration fz = kinematics.getSpecificForceZ();

        final double pitch2 = LevelingEstimator.getPitch(fx, fy, fz);

        // Kinematics estimator, also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term, for that reason amount of error is slightly larger
        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    public void testGetYaw4() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final Angle roll2 = new Angle(
                LevelingEstimator.getRoll(kinematics),
                AngleUnit.RADIANS);
        final Angle pitch2 = new Angle(
                LevelingEstimator.getPitch(kinematics),
                AngleUnit.RADIANS);
        final AngularSpeed angularRateX = kinematics.getAngularSpeedX();
        final AngularSpeed angularRateY = kinematics.getAngularSpeedY();
        final AngularSpeed angularRateZ = kinematics.getAngularSpeedZ();

        final double yaw2 = LevelingEstimator.getYaw(roll2, pitch2,
                angularRateX, angularRateY, angularRateZ);

        // Kinematics estimator, also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term, for that reason amount of error is slightly larger
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    public void testGetYaw5() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final Acceleration fx = kinematics.getSpecificForceX();
        final Acceleration fy = kinematics.getSpecificForceY();
        final Acceleration fz = kinematics.getSpecificForceZ();
        final AngularSpeed angularRateX = kinematics.getAngularSpeedX();
        final AngularSpeed angularRateY = kinematics.getAngularSpeedY();
        final AngularSpeed angularRateZ = kinematics.getAngularSpeedZ();

        final double yaw2 = LevelingEstimator.getYaw(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        // Kinematics estimator, also includes the effects of Earth
        // rotation on sensed specific force, whereas leveling neglects
        // this term, for that reason amount of error is slightly larger
        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
    }

    @Test
    public void testGetAttitude5() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final Acceleration fx = kinematics.getSpecificForceX();
        final Acceleration fy = kinematics.getSpecificForceY();
        final Acceleration fz = kinematics.getSpecificForceZ();
        final AngularSpeed angularRateX = kinematics.getAngularSpeedX();
        final AngularSpeed angularRateY = kinematics.getAngularSpeedY();
        final AngularSpeed angularRateZ = kinematics.getAngularSpeedZ();

        final CoordinateTransformation bodyC2 =
                new CoordinateTransformation(FrameType.BODY_FRAME,
                        FrameType.BODY_FRAME);

        LevelingEstimator.getAttitude(
                fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                bodyC2);

        assertTrue(bodyC.equals(bodyC2, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testGetAttitude6() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final Acceleration fx = kinematics.getSpecificForceX();
        final Acceleration fy = kinematics.getSpecificForceY();
        final Acceleration fz = kinematics.getSpecificForceZ();
        final AngularSpeed angularRateX = kinematics.getAngularSpeedX();
        final AngularSpeed angularRateY = kinematics.getAngularSpeedY();
        final AngularSpeed angularRateZ = kinematics.getAngularSpeedZ();

        final CoordinateTransformation bodyC2 =
                LevelingEstimator.getAttitude(
                        fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        assertTrue(bodyC.equals(bodyC2, LARGE_ABSOLUTE_ERROR));
    }

    @Test
    public void testGetRollAsAngle() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final double roll2 = LevelingEstimator.getRoll(
                kinematics.getFy(), kinematics.getFz());
        final Angle roll3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getRollAsAngle(kinematics.getFy(),
                kinematics.getFz(), roll3);
        final Angle roll4 = LevelingEstimator.getRollAsAngle(
                kinematics.getFy(), kinematics.getFz());

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(roll2, roll3.getValue().doubleValue(), 0.0);
        assertEquals(roll3.getUnit(), AngleUnit.RADIANS);
        assertEquals(roll3, roll4);
    }

    @Test
    public void testGetPitchAsAngle() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final double pitch2 = LevelingEstimator.getPitch(
                kinematics.getFx(), kinematics.getFy(),
                kinematics.getFz());
        final Angle pitch3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getPitchAsAngle(kinematics.getFx(),
                kinematics.getFy(), kinematics.getFz(),
                pitch3);
        final Angle pitch4 = LevelingEstimator.getPitchAsAngle(
                kinematics.getFx(), kinematics.getFy(),
                kinematics.getFz());

        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch2, pitch3.getValue().doubleValue(), 0.0);
        assertEquals(pitch3.getUnit(), AngleUnit.RADIANS);
        assertEquals(pitch3, pitch4);
    }

    @Test
    public void testGetYawAsAngle() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final double roll2 = LevelingEstimator.getRoll(kinematics.getFy(),
                kinematics.getFz());
        final double pitch2 = LevelingEstimator.getPitch(
                kinematics.getFx(), kinematics.getFy(),
                kinematics.getFz());
        final double yaw2 = LevelingEstimator.getYaw(roll2, pitch2,
                kinematics.getAngularRateX(), kinematics.getAngularRateY(),
                kinematics.getAngularRateZ());

        final Angle yaw3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getYawAsAngle(roll2, pitch2,
                kinematics.getAngularRateX(),
                kinematics.getAngularRateY(),
                kinematics.getAngularRateZ(), yaw3);
        final Angle yaw4 = LevelingEstimator.getYawAsAngle(
                roll2, pitch2,
                kinematics.getAngularRateX(),
                kinematics.getAngularRateY(),
                kinematics.getAngularRateZ());

        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw2, yaw3.getValue().doubleValue(), 0.0);
        assertEquals(yaw3.getUnit(), AngleUnit.RADIANS);
        assertEquals(yaw3, yaw4);
    }

    @Test
    public void testGetYawAsAngle2() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final double yaw2 = LevelingEstimator.getYaw(
                kinematics.getFx(), kinematics.getFy(),
                kinematics.getFz(), kinematics.getAngularRateX(),
                kinematics.getAngularRateY(),
                kinematics.getAngularRateZ());
        final Angle yaw3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getYawAsAngle(
                kinematics.getFx(),
                kinematics.getFy(),
                kinematics.getFz(),
                kinematics.getAngularRateX(),
                kinematics.getAngularRateY(),
                kinematics.getAngularRateZ(), yaw3);
        final Angle yaw4 = LevelingEstimator.getYawAsAngle(
                kinematics.getFx(),
                kinematics.getFy(),
                kinematics.getFz(),
                kinematics.getAngularRateX(),
                kinematics.getAngularRateY(),
                kinematics.getAngularRateZ());

        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw2, yaw3.getValue().doubleValue(), 0.0);
        assertEquals(yaw3.getUnit(), AngleUnit.RADIANS);
        assertEquals(yaw3, yaw4);
    }

    @Test
    public void testGetRollAsAngle2() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final double roll2 = LevelingEstimator.getRoll(kinematics);
        final Angle roll3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getRollAsAngle(kinematics, roll3);
        final Angle roll4 = LevelingEstimator.getRollAsAngle(
                kinematics);

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(roll2, roll3.getValue().doubleValue(), 0.0);
        assertEquals(roll3.getUnit(), AngleUnit.RADIANS);
        assertEquals(roll3, roll4);
    }

    @Test
    public void testGetPitchAsAngle2() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final double pitch2 = LevelingEstimator.getPitch(kinematics);
        final Angle pitch3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getPitchAsAngle(kinematics, pitch3);
        final Angle pitch4 = LevelingEstimator.getPitchAsAngle(
                kinematics);

        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch2, pitch3.getValue().doubleValue(), 0.0);
        assertEquals(pitch3.getUnit(), AngleUnit.RADIANS);
        assertEquals(pitch3, pitch4);
    }

    @Test
    public void testGetYawAsAngle3() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final double yaw2 = LevelingEstimator.getYaw(kinematics);
        final Angle yaw3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getYawAsAngle(kinematics, yaw3);
        final Angle yaw4 = LevelingEstimator.getYawAsAngle(kinematics);

        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw2, yaw3.getValue().doubleValue(), 0.0);
        assertEquals(yaw3.getUnit(), AngleUnit.RADIANS);
        assertEquals(yaw3, yaw4);
    }

    @Test
    public void testGetRollAsAngle3() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final Acceleration fy = kinematics.getSpecificForceY();
        final Acceleration fz = kinematics.getSpecificForceZ();

        final double roll2 = LevelingEstimator.getRoll(fy, fz);
        final Angle roll3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getRollAsAngle(fy, fz, roll3);
        final Angle roll4 = LevelingEstimator.getRollAsAngle(fy, fz);

        assertEquals(roll1, roll2, LARGE_ABSOLUTE_ERROR);
        assertEquals(roll2, roll3.getValue().doubleValue(), 0.0);
        assertEquals(roll3.getUnit(), AngleUnit.RADIANS);
        assertEquals(roll3, roll4);
    }

    @Test
    public void testGetPitchAsAngle3() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final Acceleration fx = kinematics.getSpecificForceX();
        final Acceleration fy = kinematics.getSpecificForceY();
        final Acceleration fz = kinematics.getSpecificForceZ();

        final double pitch2 = LevelingEstimator.getPitch(fx, fy, fz);
        final Angle pitch3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getPitchAsAngle(fx, fy, fz, pitch3);
        final Angle pitch4 = LevelingEstimator.getPitchAsAngle(
                fx, fy, fz);

        assertEquals(pitch1, pitch2, LARGE_ABSOLUTE_ERROR);
        assertEquals(pitch2, pitch3.getValue().doubleValue(), 0.0);
        assertEquals(pitch3.getUnit(), AngleUnit.RADIANS);
        assertEquals(pitch3, pitch4);
    }

    @Test
    public void testGetYawAsAngle4() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final Angle roll2 = new Angle(
                LevelingEstimator.getRoll(kinematics),
                AngleUnit.RADIANS);
        final Angle pitch2 = new Angle(
                LevelingEstimator.getPitch(kinematics),
                AngleUnit.RADIANS);
        final AngularSpeed angularRateX = kinematics.getAngularSpeedX();
        final AngularSpeed angularRateY = kinematics.getAngularSpeedY();
        final AngularSpeed angularRateZ = kinematics.getAngularSpeedZ();

        final double yaw2 = LevelingEstimator.getYaw(roll2, pitch2,
                angularRateX, angularRateY, angularRateZ);
        final Angle yaw3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getYawAsAngle(roll2, pitch2, angularRateX,
                angularRateY, angularRateZ, yaw3);
        final Angle yaw4 = LevelingEstimator.getYawAsAngle(roll2,
                pitch2, angularRateX, angularRateY, angularRateZ);

        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw2, yaw3.getValue().doubleValue(), 0.0);
        assertEquals(yaw3.getUnit(), AngleUnit.RADIANS);
        assertEquals(yaw3, yaw4);
    }

    @Test
    public void testGetYawAsAngle5() {
        final UniformRandomizer randomizer =
                new UniformRandomizer(new Random());
        final double latitude = Math.toRadians(
                randomizer.nextDouble(MIN_LATITUDE_DEGREES,
                        MAX_LATITUDE_DEGREES));
        final double height = randomizer.nextDouble(
                MIN_HEIGHT, MAX_HEIGHT);

        // body attitude
        final double roll1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double pitch1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));
        final double yaw1 = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES,
                        MAX_ANGLE_DEGREES));

        // attitude is expressed as rotation from local navigation frame
        // to body frame, since angles are measured on the device body
        final CoordinateTransformation bodyC = new CoordinateTransformation(
                roll1, pitch1, yaw1, FrameType.LOCAL_NAVIGATION_FRAME,
                FrameType.BODY_FRAME);
        final CoordinateTransformation nedC = bodyC.inverseAndReturnNew();

        // obtain expected kinematics measure
        final BodyKinematics kinematics = NEDKinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL,
                        nedC, nedC,
                        0.0, 0.0, 0.0,
                        0.0, 0.0, 0.0,
                        latitude, height,
                        latitude, height);

        final Acceleration fx = kinematics.getSpecificForceX();
        final Acceleration fy = kinematics.getSpecificForceY();
        final Acceleration fz = kinematics.getSpecificForceZ();
        final AngularSpeed angularRateX = kinematics.getAngularSpeedX();
        final AngularSpeed angularRateY = kinematics.getAngularSpeedY();
        final AngularSpeed angularRateZ = kinematics.getAngularSpeedZ();

        final double yaw2 = LevelingEstimator.getYaw(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
        final Angle yaw3 = new Angle(0.0, AngleUnit.DEGREES);
        LevelingEstimator.getYawAsAngle(fx, fy, fz, angularRateX,
                angularRateY, angularRateZ, yaw3);
        final Angle yaw4 = LevelingEstimator.getYawAsAngle(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        assertEquals(yaw1, yaw2, LARGE_ABSOLUTE_ERROR);
        assertEquals(yaw2, yaw3.getValue().doubleValue(), 0.0);
        assertEquals(yaw3.getUnit(), AngleUnit.RADIANS);
        assertEquals(yaw3, yaw4);
    }
}
