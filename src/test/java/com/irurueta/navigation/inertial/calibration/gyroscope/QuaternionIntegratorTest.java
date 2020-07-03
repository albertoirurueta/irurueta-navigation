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

import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.RotationException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator;
import com.irurueta.navigation.inertial.navigators.InertialNavigatorException;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.assertTrue;

public class QuaternionIntegratorTest {

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT = -100.0;
    private static final double MAX_HEIGHT = 10000.0;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    // assume 50 samples per second
    private static final int NUM_SAMPLES = 50;
    private static final double TIME_INTERVAL = 1.0;
    private static final double TIME_INTERVAL_BETWEEN_SAMPLES = TIME_INTERVAL / NUM_SAMPLES;

    private static final double ABSOLUTE_ERROR = 1e-2;

    private static final int TIMES = 100;

    @Test
    public void testIntegrateGyroSequence1WithInitialAttitude()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException,
            RotationException, InertialNavigatorException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double latitude = Math.toRadians(randomizer.nextDouble(
                    MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(randomizer.nextDouble(
                    MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double roll1 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch1 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw1 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final CoordinateTransformation cbn1 = new CoordinateTransformation(
                    roll1, pitch1, yaw1, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame1 = new NEDFrame(nedPosition, cbn1);
            final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // compute new orientation after 1 second
            final double roll2 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch2 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw2 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final CoordinateTransformation cbn2 = new CoordinateTransformation(
                    roll2, pitch2, yaw2, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame2 = new NEDFrame(nedPosition, cbn2);
            final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            // assume that body kinematics are constant during the time interval of 1 second
            final BodyKinematics kinematics = ECEFKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL, ecefFrame2, ecefFrame1);


            final ECEFFrame previousFrame = new ECEFFrame(ecefFrame1);
            final ECEFFrame currentFrame = new ECEFFrame();

            final List<StandardDeviationTimedBodyKinematics> items = new ArrayList<>();
            for (int i = 0; i < NUM_SAMPLES; i++) {
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_BETWEEN_SAMPLES, previousFrame, kinematics,
                        currentFrame);

                previousFrame.copyFrom(currentFrame);

                final double timestamp = i * TIME_INTERVAL_BETWEEN_SAMPLES;
                final StandardDeviationTimedBodyKinematics item =
                        new StandardDeviationTimedBodyKinematics(kinematics, timestamp);
                items.add(item);
            }

            final Quaternion expected = currentFrame.getCoordinateTransformation()
                    .asRotation().toQuaternion();
            expected.normalize();

            final Quaternion expected2 = ecefFrame2.getCoordinateTransformation()
                    .asRotation().toQuaternion();
            expected2.normalize();

            assertTrue(expected.equals(expected2, ABSOLUTE_ERROR));

            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                    new BodyKinematicsSequence<>(items);
            final Quaternion initialAttitude = ecefFrame1.getCoordinateTransformation()
                    .asRotation().toQuaternion();
            final Quaternion result = new Quaternion();

            QuaternionIntegrator.integrateGyroSequence(sequence, initialAttitude, result);
            result.normalize();

            if (!result.equals(expected, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(result.equals(expected, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testIntegrateGyroSequence1WithNoInitialAttitude()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException,
            RotationException, InertialNavigatorException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double latitude = Math.toRadians(randomizer.nextDouble(
                    MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(randomizer.nextDouble(
                    MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double roll1 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch1 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw1 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final CoordinateTransformation cbn1 = new CoordinateTransformation(
                    roll1, pitch1, yaw1, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame1 = new NEDFrame(nedPosition, cbn1);
            final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // compute new orientation after 1 second
            final double roll2 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch2 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw2 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final CoordinateTransformation cbn2 = new CoordinateTransformation(
                    roll2, pitch2, yaw2, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame2 = new NEDFrame(nedPosition, cbn2);
            final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            // assume that body kinematics are constant during the time interval of 1 second
            final BodyKinematics kinematics = ECEFKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL, ecefFrame2, ecefFrame1);


            final ECEFFrame previousFrame = new ECEFFrame(ecefFrame1);
            // set the identity ad initial attitude
            previousFrame.setCoordinateTransformation(
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME));

            final ECEFFrame currentFrame = new ECEFFrame();

            final List<StandardDeviationTimedBodyKinematics> items = new ArrayList<>();
            for (int i = 0; i < NUM_SAMPLES; i++) {
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_BETWEEN_SAMPLES, previousFrame, kinematics,
                        currentFrame);

                previousFrame.copyFrom(currentFrame);

                final double timestamp = i * TIME_INTERVAL_BETWEEN_SAMPLES;
                final StandardDeviationTimedBodyKinematics item =
                        new StandardDeviationTimedBodyKinematics(kinematics, timestamp);
                items.add(item);
            }

            final Quaternion expected = currentFrame.getCoordinateTransformation()
                    .asRotation().toQuaternion();
            expected.normalize();

            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                    new BodyKinematicsSequence<>(items);
            final Quaternion result = new Quaternion();

            QuaternionIntegrator.integrateGyroSequence(sequence, null, result);
            result.normalize();

            if (!result.equals(expected, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(result.equals(expected, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testIntegrateGyroSequence2()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException,
            RotationException, InertialNavigatorException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double latitude = Math.toRadians(randomizer.nextDouble(
                    MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(randomizer.nextDouble(
                    MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double roll1 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch1 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw1 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final CoordinateTransformation cbn1 = new CoordinateTransformation(
                    roll1, pitch1, yaw1, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame1 = new NEDFrame(nedPosition, cbn1);
            final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // compute new orientation after 1 second
            final double roll2 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch2 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw2 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final CoordinateTransformation cbn2 = new CoordinateTransformation(
                    roll2, pitch2, yaw2, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame2 = new NEDFrame(nedPosition, cbn2);
            final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            // assume that body kinematics are constant during the time interval of 1 second
            final BodyKinematics kinematics = ECEFKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL, ecefFrame2, ecefFrame1);


            final ECEFFrame previousFrame = new ECEFFrame(ecefFrame1);
            // set the identity ad initial attitude
            previousFrame.setCoordinateTransformation(
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME));

            final ECEFFrame currentFrame = new ECEFFrame();

            final List<StandardDeviationTimedBodyKinematics> items = new ArrayList<>();
            for (int i = 0; i < NUM_SAMPLES; i++) {
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_BETWEEN_SAMPLES, previousFrame, kinematics,
                        currentFrame);

                previousFrame.copyFrom(currentFrame);

                final double timestamp = i * TIME_INTERVAL_BETWEEN_SAMPLES;
                final StandardDeviationTimedBodyKinematics item =
                        new StandardDeviationTimedBodyKinematics(kinematics, timestamp);
                items.add(item);
            }

            final Quaternion expected = currentFrame.getCoordinateTransformation()
                    .asRotation().toQuaternion();
            expected.normalize();

            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                    new BodyKinematicsSequence<>(items);
            final Quaternion result = new Quaternion();

            QuaternionIntegrator.integrateGyroSequence(sequence, result);
            result.normalize();

            if (!result.equals(expected, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(result.equals(expected, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testIntegrateGyroSequenceAndReturnNew1()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException,
            RotationException, InertialNavigatorException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double latitude = Math.toRadians(randomizer.nextDouble(
                    MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(randomizer.nextDouble(
                    MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double roll1 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch1 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw1 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final CoordinateTransformation cbn1 = new CoordinateTransformation(
                    roll1, pitch1, yaw1, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame1 = new NEDFrame(nedPosition, cbn1);
            final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // compute new orientation after 1 second
            final double roll2 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch2 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw2 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final CoordinateTransformation cbn2 = new CoordinateTransformation(
                    roll2, pitch2, yaw2, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame2 = new NEDFrame(nedPosition, cbn2);
            final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            // assume that body kinematics are constant during the time interval of 1 second
            final BodyKinematics kinematics = ECEFKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL, ecefFrame2, ecefFrame1);


            final ECEFFrame previousFrame = new ECEFFrame(ecefFrame1);
            final ECEFFrame currentFrame = new ECEFFrame();

            final List<StandardDeviationTimedBodyKinematics> items = new ArrayList<>();
            for (int i = 0; i < NUM_SAMPLES; i++) {
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_BETWEEN_SAMPLES, previousFrame, kinematics,
                        currentFrame);

                previousFrame.copyFrom(currentFrame);

                final double timestamp = i * TIME_INTERVAL_BETWEEN_SAMPLES;
                final StandardDeviationTimedBodyKinematics item =
                        new StandardDeviationTimedBodyKinematics(kinematics, timestamp);
                items.add(item);
            }

            final Quaternion expected = currentFrame.getCoordinateTransformation()
                    .asRotation().toQuaternion();
            expected.normalize();

            final Quaternion expected2 = ecefFrame2.getCoordinateTransformation()
                    .asRotation().toQuaternion();
            expected2.normalize();

            assertTrue(expected.equals(expected2, ABSOLUTE_ERROR));

            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                    new BodyKinematicsSequence<>(items);
            final Quaternion initialAttitude = ecefFrame1.getCoordinateTransformation()
                    .asRotation().toQuaternion();
            final Quaternion result = QuaternionIntegrator
                    .integrateGyroSequenceAndReturnNew(sequence,
                            initialAttitude);
            result.normalize();

            if (!result.equals(expected, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(result.equals(expected, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testIntegrateGyroSequenceAndReturnNew2()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException,
            RotationException, InertialNavigatorException {
        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());
            final double latitude = Math.toRadians(randomizer.nextDouble(
                    MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
            final double longitude = Math.toRadians(randomizer.nextDouble(
                    MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
            final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

            final double roll1 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch1 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw1 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final CoordinateTransformation cbn1 = new CoordinateTransformation(
                    roll1, pitch1, yaw1, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame1 = new NEDFrame(nedPosition, cbn1);
            final ECEFFrame ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            // compute new orientation after 1 second
            final double roll2 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double pitch2 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double yaw2 = Math.toRadians(randomizer.nextDouble(
                    MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final CoordinateTransformation cbn2 = new CoordinateTransformation(
                    roll2, pitch2, yaw2, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

            final NEDFrame nedFrame2 = new NEDFrame(nedPosition, cbn2);
            final ECEFFrame ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            // assume that body kinematics are constant during the time interval of 1 second
            final BodyKinematics kinematics = ECEFKinematicsEstimator
                    .estimateKinematicsAndReturnNew(TIME_INTERVAL, ecefFrame2, ecefFrame1);


            final ECEFFrame previousFrame = new ECEFFrame(ecefFrame1);
            // set the identity as initial attitude
            previousFrame.setCoordinateTransformation(
                    new CoordinateTransformation(FrameType.BODY_FRAME,
                            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME));

            final ECEFFrame currentFrame = new ECEFFrame();

            final List<StandardDeviationTimedBodyKinematics> items = new ArrayList<>();
            for (int i = 0; i < NUM_SAMPLES; i++) {
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_BETWEEN_SAMPLES, previousFrame, kinematics,
                        currentFrame);

                previousFrame.copyFrom(currentFrame);

                final double timestamp = i * TIME_INTERVAL_BETWEEN_SAMPLES;
                final StandardDeviationTimedBodyKinematics item =
                        new StandardDeviationTimedBodyKinematics(kinematics, timestamp);
                items.add(item);
            }

            final Quaternion expected = currentFrame.getCoordinateTransformation()
                    .asRotation().toQuaternion();
            expected.normalize();

            final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence =
                    new BodyKinematicsSequence<>(items);
            final Quaternion result = QuaternionIntegrator
                    .integrateGyroSequenceAndReturnNew(sequence);
            result.normalize();

            if (!result.equals(expected, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(result.equals(expected, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

}
