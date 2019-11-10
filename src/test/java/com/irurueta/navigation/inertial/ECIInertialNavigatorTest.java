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
package com.irurueta.navigation.inertial;

import com.irurueta.algebra.Matrix;
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
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.ECItoECEFFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.*;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class ECIInertialNavigatorTest {
    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double LATITUDE_DEGREES = 41.3825;
    private static final double LONGITUDE_DEGREES = 2.176944;
    private static final double HEIGHT = 0.0;


    private static final double MIN_HEIGHT = -10.0;
    private static final double MAX_HEIGHT = 10.0;

    private static final double MIN_VELOCITY_VALUE = -2.0;
    private static final double MAX_VELOCITY_VALUE = 2.0;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_SPECIFIC_FORCE = -12.0;
    private static final double MAX_SPECIFIC_FORCE = 12.0;

    private static final double MIN_ANGULAR_RATE_DEGREES_PER_SECOND = -5.0;
    private static final double MAX_ANGULAR_RATE_DEGREES_PER_SECOND = 5.0;

    private static final double ABSOLUTE_ERROR = 1e-5;

    private static final int TIMES = 100;

    @Test(expected = InvalidSourceAndDestinationFrameTypeException.class)
    public void testNavigateECiWhenInvalidCoordinateTransformationMatrix()
            throws InvalidSourceAndDestinationFrameTypeException,
            InertialNavigatorException {

        final CoordinateTransformation c = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        final ECIFrame result = new ECIFrame();
        ECIInertialNavigator.navigateECI(0.0,
                0.0, 0.0, 0.0, c,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, result);
    }

    @Test
    public void testNavigate() throws InvalidRotationMatrixException, InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {
        final ECIInertialNavigator navigator = new ECIInertialNavigator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double latitude = Math.toRadians(LATITUDE_DEGREES);
        final double longitude = Math.toRadians(LONGITUDE_DEGREES);

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
                m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame oldNedFrame = new NEDFrame(latitude, longitude, HEIGHT,
                vn, ve, vd, c);
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(oldNedFrame);
        final ECIFrame oldFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(
                TIME_INTERVAL_SECONDS, oldEcefFrame);

        final double oldX = oldFrame.getX();
        final double oldY = oldFrame.getY();
        final double oldZ = oldFrame.getZ();
        final CoordinateTransformation oldC = oldFrame.getCoordinateTransformation();
        final double oldVx = oldFrame.getVx();
        final double oldVy = oldFrame.getVy();
        final double oldVz = oldFrame.getVz();

        final double fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final double angularRateXDegreesPerSecond = randomizer.nextDouble(
                MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
        final double angularRateYDegreesPerSecond = randomizer.nextDouble(
                MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
        final double angularRateZDegreesPerSecond = randomizer.nextDouble(
                MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);

        final AngularSpeed angularSpeedX = new AngularSpeed(angularRateXDegreesPerSecond,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        final AngularSpeed angularSpeedY = new AngularSpeed(angularRateYDegreesPerSecond,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        final AngularSpeed angularSpeedZ = new AngularSpeed(angularRateZDegreesPerSecond,
                AngularSpeedUnit.DEGREES_PER_SECOND);

        final double angularRateX = AngularSpeedConverter.convert(
                angularSpeedX.getValue().doubleValue(), angularSpeedX.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final double angularRateY = AngularSpeedConverter.convert(
                angularSpeedY.getValue().doubleValue(), angularSpeedY.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final double angularRateZ = AngularSpeedConverter.convert(
                angularSpeedZ.getValue().doubleValue(), angularSpeedZ.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);

        final BodyKinematics kinematics = new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECIFrame result1 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result1);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final ECIFrame result2 = new ECIFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result2);

        final ECIFrame result3 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, kinematics, result3);

        final ECIFrame result4 = new ECIFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, kinematics, result4);

        final Point3D oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);
        final ECIFrame result5 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result5);

        final ECIFrame result6 = new ECIFrame();
        navigator.navigate(timeInterval, oldPosition,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result6);

        final ECIFrame result7 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition,
                oldC, oldVx, oldVy, oldVz, kinematics, result7);

        final ECIFrame result8 = new ECIFrame();
        navigator.navigate(timeInterval, oldPosition,
                oldC, oldVx, oldVy, oldVz, kinematics, result8);

        final Distance oldDistanceX = new Distance(oldX, DistanceUnit.METER);
        final Distance oldDistanceY = new Distance(oldY, DistanceUnit.METER);
        final Distance oldDistanceZ = new Distance(oldZ, DistanceUnit.METER);
        final ECIFrame result9 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result9);

        final ECIFrame result10 = new ECIFrame();
        navigator.navigate(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result10);

        final ECIFrame result11 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics, result11);

        final ECIFrame result12 = new ECIFrame();
        navigator.navigate(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics, result12);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);
        final ECIFrame result13 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result13);

        final ECIFrame result14 = new ECIFrame();
        navigator.navigate(timeInterval,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result14);

        final ECIFrame result15 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result15);

        final ECIFrame result16 = new ECIFrame();
        navigator.navigate(timeInterval,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result16);

        final Acceleration accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final ECIFrame result17 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result17);

        final ECIFrame result18 = new ECIFrame();
        navigator.navigate(timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result18);

        final ECIFrame result19 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result19);

        final ECIFrame result20 = new ECIFrame();
        navigator.navigate(timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result20);

        final ECIFrame result21 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result21);

        final ECIFrame result22 = new ECIFrame();
        navigator.navigate(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result22);

        final ECIFrame result23 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result23);

        final ECIFrame result24 = new ECIFrame();
        navigator.navigate(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result24);

        final ECIFrame result25 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result25);

        final ECIFrame result26 = new ECIFrame();
        navigator.navigate(timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result26);

        final ECIFrame result27 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics,
                result27);

        final ECIFrame result28 = new ECIFrame();
        navigator.navigate(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics,
                result28);

        final ECIFrame result29 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result29);

        final ECIFrame result30 = new ECIFrame();
        navigator.navigate(timeInterval,
                oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result30);

        final ECIFrame result31 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldFrame, kinematics, result31);

        final ECIFrame result32 = new ECIFrame();
        navigator.navigate(timeInterval,
                oldFrame, kinematics, result32);

        final ECIFrame result33 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result33);

        final ECIFrame result34 = new ECIFrame();
        navigator.navigate(timeInterval,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result34);

        final ECIFrame result35 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result35);

        final ECIFrame result36 = new ECIFrame();
        navigator.navigate(timeInterval,
                oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result36);

        final ECIFrame result37 = new ECIFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result37);

        final ECIFrame result38 = new ECIFrame();
        navigator.navigate(timeInterval,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result38);

        assertEquals(result1, result2);
        assertEquals(result1, result3);
        assertEquals(result1, result4);
        assertEquals(result1, result5);
        assertEquals(result1, result6);
        assertEquals(result1, result7);
        assertEquals(result1, result8);
        assertEquals(result1, result9);
        assertEquals(result1, result10);
        assertEquals(result1, result11);
        assertEquals(result1, result12);
        assertEquals(result1, result13);
        assertEquals(result1, result14);
        assertEquals(result1, result15);
        assertEquals(result1, result16);
        assertEquals(result1, result17);
        assertEquals(result1, result18);
        assertEquals(result1, result19);
        assertEquals(result1, result20);
        assertEquals(result1, result21);
        assertEquals(result1, result22);
        assertEquals(result1, result23);
        assertEquals(result1, result24);
        assertEquals(result1, result25);
        assertEquals(result1, result26);
        assertEquals(result1, result27);
        assertEquals(result1, result28);
        assertEquals(result1, result29);
        assertEquals(result1, result30);
        assertEquals(result1, result31);
        assertEquals(result1, result32);
        assertEquals(result1, result33);
        assertEquals(result1, result34);
        assertEquals(result1, result35);
        assertEquals(result1, result36);
        assertEquals(result1, result37);
        assertEquals(result1, result38);
    }

    @Test
    public void testNavigateAndReturnNew() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {
        final ECIInertialNavigator navigator = new ECIInertialNavigator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double latitude = Math.toRadians(LATITUDE_DEGREES);
        final double longitude = Math.toRadians(LONGITUDE_DEGREES);

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

        final NEDFrame oldNedFrame = new NEDFrame(latitude, longitude, HEIGHT, vn, ve, vd, c);
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
        final ECIFrame oldFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(
                TIME_INTERVAL_SECONDS, oldEcefFrame);

        final double oldX = oldFrame.getX();
        final double oldY = oldFrame.getY();
        final double oldZ = oldFrame.getZ();
        final CoordinateTransformation oldC = oldFrame.getCoordinateTransformation();
        final double oldVx = oldFrame.getVx();
        final double oldVy = oldFrame.getVy();
        final double oldVz = oldFrame.getVz();


        final double fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final double angularRateXDegreesPerSecond = randomizer.nextDouble(
                MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
        final double angularRateYDegreesPerSecond = randomizer.nextDouble(
                MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
        final double angularRateZDegreesPerSecond = randomizer.nextDouble(
                MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);

        final AngularSpeed angularSpeedX = new AngularSpeed(angularRateXDegreesPerSecond,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        final AngularSpeed angularSpeedY = new AngularSpeed(angularRateYDegreesPerSecond,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        final AngularSpeed angularSpeedZ = new AngularSpeed(angularRateZDegreesPerSecond,
                AngularSpeedUnit.DEGREES_PER_SECOND);

        final double angularRateX = AngularSpeedConverter.convert(
                angularSpeedX.getValue().doubleValue(), angularSpeedX.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final double angularRateY = AngularSpeedConverter.convert(
                angularSpeedY.getValue().doubleValue(), angularSpeedY.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final double angularRateZ = AngularSpeedConverter.convert(
                angularSpeedZ.getValue().doubleValue(), angularSpeedZ.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);

        final BodyKinematics kinematics = new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECIFrame result1 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final ECIFrame result2 = navigator.navigateAndReturnNew(
                timeInterval, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECIFrame result3 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, kinematics);

        final ECIFrame result4 = navigator.navigateAndReturnNew(
                timeInterval, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, kinematics);

        final Point3D oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);
        final ECIFrame result5 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECIFrame result6 = navigator.navigateAndReturnNew(
                timeInterval, oldPosition,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECIFrame result7 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition,
                oldC, oldVx, oldVy, oldVz, kinematics);

        final ECIFrame result8 = navigator.navigateAndReturnNew(
                timeInterval, oldPosition,
                oldC, oldVx, oldVy, oldVz, kinematics);

        final Distance oldDistanceX = new Distance(oldX, DistanceUnit.METER);
        final Distance oldDistanceY = new Distance(oldY, DistanceUnit.METER);
        final Distance oldDistanceZ = new Distance(oldZ, DistanceUnit.METER);
        final ECIFrame result9 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECIFrame result10 = navigator.navigateAndReturnNew(
                timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECIFrame result11 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics);

        final ECIFrame result12 = navigator.navigateAndReturnNew(
                timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);
        final ECIFrame result13 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final ECIFrame result14 = navigator.navigateAndReturnNew(
                timeInterval,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final ECIFrame result15 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics);

        final ECIFrame result16 = navigator.navigateAndReturnNew(
                timeInterval,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics);

        final Acceleration accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final ECIFrame result17 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final ECIFrame result18 = navigator.navigateAndReturnNew(
                timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final ECIFrame result19 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECIFrame result20 = navigator.navigateAndReturnNew(
                timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECIFrame result21 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECIFrame result22 = navigator.navigateAndReturnNew(
                timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECIFrame result23 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECIFrame result24 = navigator.navigateAndReturnNew(
                timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECIFrame result25 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECIFrame result26 = navigator.navigateAndReturnNew(
                timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECIFrame result27 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final ECIFrame result28 = navigator.navigateAndReturnNew(
                timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final ECIFrame result29 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECIFrame result30 = navigator.navigateAndReturnNew(
                timeInterval,
                oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECIFrame result31 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, kinematics);

        final ECIFrame result32 = navigator.navigateAndReturnNew(
                timeInterval, oldFrame, kinematics);

        final ECIFrame result33 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final ECIFrame result34 = navigator.navigateAndReturnNew(
                timeInterval,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final ECIFrame result35 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECIFrame result36 = navigator.navigateAndReturnNew(
                timeInterval,
                oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECIFrame result37 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECIFrame result38 = navigator.navigateAndReturnNew(
                timeInterval,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        assertEquals(result1, result2);
        assertEquals(result1, result3);
        assertEquals(result1, result4);
        assertEquals(result1, result5);
        assertEquals(result1, result6);
        assertEquals(result1, result7);
        assertEquals(result1, result8);
        assertEquals(result1, result9);
        assertEquals(result1, result10);
        assertEquals(result1, result11);
        assertEquals(result1, result12);
        assertEquals(result1, result13);
        assertEquals(result1, result14);
        assertEquals(result1, result15);
        assertEquals(result1, result16);
        assertEquals(result1, result17);
        assertEquals(result1, result18);
        assertEquals(result1, result19);
        assertEquals(result1, result20);
        assertEquals(result1, result21);
        assertEquals(result1, result22);
        assertEquals(result1, result23);
        assertEquals(result1, result24);
        assertEquals(result1, result25);
        assertEquals(result1, result26);
        assertEquals(result1, result27);
        assertEquals(result1, result28);
        assertEquals(result1, result29);
        assertEquals(result1, result30);
        assertEquals(result1, result31);
        assertEquals(result1, result32);
        assertEquals(result1, result33);
        assertEquals(result1, result34);
        assertEquals(result1, result35);
        assertEquals(result1, result36);
        assertEquals(result1, result37);
        assertEquals(result1, result38);
    }

    @Test
    public void testNavigateEci() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double latitude = Math.toRadians(LATITUDE_DEGREES);
        final double longitude = Math.toRadians(LONGITUDE_DEGREES);

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

        final NEDFrame oldNedFrame = new NEDFrame(latitude, longitude, HEIGHT, vn, ve, vd, c);
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(oldNedFrame);
        final ECIFrame oldFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(
                TIME_INTERVAL_SECONDS, oldEcefFrame);

        final double oldX = oldFrame.getX();
        final double oldY = oldFrame.getY();
        final double oldZ = oldFrame.getZ();
        final CoordinateTransformation oldC = oldFrame.getCoordinateTransformation();
        final double oldVx = oldFrame.getVx();
        final double oldVy = oldFrame.getVy();
        final double oldVz = oldFrame.getVz();


        final double fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final double angularRateXDegreesPerSecond = randomizer.nextDouble(
                MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
        final double angularRateYDegreesPerSecond = randomizer.nextDouble(
                MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
        final double angularRateZDegreesPerSecond = randomizer.nextDouble(
                MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);

        final AngularSpeed angularSpeedX = new AngularSpeed(angularRateXDegreesPerSecond,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        final AngularSpeed angularSpeedY = new AngularSpeed(angularRateYDegreesPerSecond,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        final AngularSpeed angularSpeedZ = new AngularSpeed(angularRateZDegreesPerSecond,
                AngularSpeedUnit.DEGREES_PER_SECOND);

        final double angularRateX = AngularSpeedConverter.convert(
                angularSpeedX.getValue().doubleValue(), angularSpeedX.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final double angularRateY = AngularSpeedConverter.convert(
                angularSpeedY.getValue().doubleValue(), angularSpeedY.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final double angularRateZ = AngularSpeedConverter.convert(
                angularSpeedZ.getValue().doubleValue(), angularSpeedZ.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);

        final BodyKinematics kinematics = new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECIFrame result1 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result1);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final ECIFrame result2 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result2);

        final ECIFrame result3 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, kinematics, result3);

        final ECIFrame result4 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, kinematics, result4);

        final Point3D oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);
        final ECIFrame result5 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldPosition,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result5);

        final ECIFrame result6 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldPosition,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result6);

        final ECIFrame result7 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS, oldPosition,
                oldC, oldVx, oldVy, oldVz, kinematics, result7);

        final ECIFrame result8 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval, oldPosition,
                oldC, oldVx, oldVy, oldVz, kinematics, result8);

        final Distance oldDistanceX = new Distance(oldX, DistanceUnit.METER);
        final Distance oldDistanceY = new Distance(oldY, DistanceUnit.METER);
        final Distance oldDistanceZ = new Distance(oldZ, DistanceUnit.METER);
        final ECIFrame result9 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result9);

        final ECIFrame result10 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result10);

        final ECIFrame result11 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics, result11);

        final ECIFrame result12 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics, result12);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);
        final ECIFrame result13 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result13);

        final ECIFrame result14 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result14);

        final ECIFrame result15 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result15);

        final ECIFrame result16 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result16);

        final Acceleration accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final ECIFrame result17 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result17);

        final ECIFrame result18 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result18);

        final ECIFrame result19 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result19);

        final ECIFrame result20 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result20);

        final ECIFrame result21 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result21);

        final ECIFrame result22 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result22);

        final ECIFrame result23 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result23);

        final ECIFrame result24 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result24);

        final ECIFrame result25 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result25);

        final ECIFrame result26 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result26);

        final ECIFrame result27 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics,
                result27);

        final ECIFrame result28 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics,
                result28);

        final ECIFrame result29 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS,
                oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result29);

        final ECIFrame result30 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval,
                oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result30);

        final ECIFrame result31 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS,
                oldFrame, kinematics, result31);

        final ECIFrame result32 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval,
                oldFrame, kinematics, result32);

        final ECIFrame result33 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result33);

        final ECIFrame result34 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result34);

        final ECIFrame result35 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS,
                oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result35);

        final ECIFrame result36 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval,
                oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result36);

        final ECIFrame result37 = new ECIFrame();
        ECIInertialNavigator.navigateECI(TIME_INTERVAL_SECONDS,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result37);

        final ECIFrame result38 = new ECIFrame();
        ECIInertialNavigator.navigateECI(timeInterval,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result38);

        assertEquals(result1, result2);
        assertEquals(result1, result3);
        assertEquals(result1, result4);
        assertEquals(result1, result5);
        assertEquals(result1, result6);
        assertEquals(result1, result7);
        assertEquals(result1, result8);
        assertEquals(result1, result9);
        assertEquals(result1, result10);
        assertEquals(result1, result11);
        assertEquals(result1, result12);
        assertEquals(result1, result13);
        assertEquals(result1, result14);
        assertEquals(result1, result15);
        assertEquals(result1, result16);
        assertEquals(result1, result17);
        assertEquals(result1, result18);
        assertEquals(result1, result19);
        assertEquals(result1, result20);
        assertEquals(result1, result21);
        assertEquals(result1, result22);
        assertEquals(result1, result23);
        assertEquals(result1, result24);
        assertEquals(result1, result25);
        assertEquals(result1, result26);
        assertEquals(result1, result27);
        assertEquals(result1, result28);
        assertEquals(result1, result29);
        assertEquals(result1, result30);
        assertEquals(result1, result31);
        assertEquals(result1, result32);
        assertEquals(result1, result33);
        assertEquals(result1, result34);
        assertEquals(result1, result35);
        assertEquals(result1, result36);
        assertEquals(result1, result37);
        assertEquals(result1, result38);
    }

    @Test
    public void testNavigateEciAndReturnNew() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {
        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double latitude = Math.toRadians(LATITUDE_DEGREES);
        final double longitude = Math.toRadians(LONGITUDE_DEGREES);

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

        final NEDFrame oldNedFrame = new NEDFrame(latitude, longitude, HEIGHT, vn, ve, vd, c);
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(oldNedFrame);
        final ECIFrame oldFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(
                TIME_INTERVAL_SECONDS, oldEcefFrame);

        final double oldX = oldFrame.getX();
        final double oldY = oldFrame.getY();
        final double oldZ = oldFrame.getZ();
        final CoordinateTransformation oldC = oldFrame.getCoordinateTransformation();
        final double oldVx = oldFrame.getVx();
        final double oldVy = oldFrame.getVy();
        final double oldVz = oldFrame.getVz();


        final double fx = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fy = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);
        final double fz = randomizer.nextDouble(MIN_SPECIFIC_FORCE, MAX_SPECIFIC_FORCE);

        final double angularRateXDegreesPerSecond = randomizer.nextDouble(
                MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
        final double angularRateYDegreesPerSecond = randomizer.nextDouble(
                MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
        final double angularRateZDegreesPerSecond = randomizer.nextDouble(
                MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                MAX_ANGULAR_RATE_DEGREES_PER_SECOND);

        final AngularSpeed angularSpeedX = new AngularSpeed(angularRateXDegreesPerSecond,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        final AngularSpeed angularSpeedY = new AngularSpeed(angularRateYDegreesPerSecond,
                AngularSpeedUnit.DEGREES_PER_SECOND);
        final AngularSpeed angularSpeedZ = new AngularSpeed(angularRateZDegreesPerSecond,
                AngularSpeedUnit.DEGREES_PER_SECOND);

        final double angularRateX = AngularSpeedConverter.convert(
                angularSpeedX.getValue().doubleValue(), angularSpeedX.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final double angularRateY = AngularSpeedConverter.convert(
                angularSpeedY.getValue().doubleValue(), angularSpeedY.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);
        final double angularRateZ = AngularSpeedConverter.convert(
                angularSpeedZ.getValue().doubleValue(), angularSpeedZ.getUnit(),
                AngularSpeedUnit.RADIANS_PER_SECOND);

        final BodyKinematics kinematics = new BodyKinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECIFrame result1 = ECIInertialNavigator
                .navigateECIAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                        oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final ECIFrame result2 = ECIInertialNavigator
                .navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ,
                        oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECIFrame result3 = ECIInertialNavigator
                .navigateECIAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                        oldC, oldVx, oldVy, oldVz, kinematics);

        final ECIFrame result4 = ECIInertialNavigator
                .navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ,
                        oldC, oldVx, oldVy, oldVz, kinematics);

        final Point3D oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);
        final ECIFrame result5 = ECIInertialNavigator
                .navigateECIAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition,
                        oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECIFrame result6 = ECIInertialNavigator
                .navigateECIAndReturnNew(timeInterval, oldPosition,
                        oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECIFrame result7 = ECIInertialNavigator
                .navigateECIAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition,
                        oldC, oldVx, oldVy, oldVz, kinematics);

        final ECIFrame result8 = ECIInertialNavigator
                .navigateECIAndReturnNew(timeInterval, oldPosition,
                        oldC, oldVx, oldVy, oldVz, kinematics);

        final Distance oldDistanceX = new Distance(oldX, DistanceUnit.METER);
        final Distance oldDistanceY = new Distance(oldY, DistanceUnit.METER);
        final Distance oldDistanceZ = new Distance(oldZ, DistanceUnit.METER);
        final ECIFrame result9 = ECIInertialNavigator
                .navigateECIAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldVx, oldVy, oldVz, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECIFrame result10 = ECIInertialNavigator
                .navigateECIAndReturnNew(timeInterval,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldVx, oldVy, oldVz, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECIFrame result11 = ECIInertialNavigator
                .navigateECIAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldVx, oldVy, oldVz, kinematics);

        final ECIFrame result12 = ECIInertialNavigator
                .navigateECIAndReturnNew(timeInterval,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldVx, oldVy, oldVz, kinematics);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);
        final ECIFrame result13 = ECIInertialNavigator
                .navigateECIAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                        fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final ECIFrame result14 = ECIInertialNavigator
                .navigateECIAndReturnNew(timeInterval,
                        oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                        fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final ECIFrame result15 = ECIInertialNavigator
                .navigateECIAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                        kinematics);

        final ECIFrame result16 = ECIInertialNavigator
                .navigateECIAndReturnNew(timeInterval,
                        oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                        kinematics);

        final Acceleration accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final ECIFrame result17 = ECIInertialNavigator
                .navigateECIAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                        accelerationX, accelerationY, accelerationZ,
                        angularRateX, angularRateY, angularRateZ);

        final ECIFrame result18 = ECIInertialNavigator
                .navigateECIAndReturnNew(timeInterval,
                        oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                        accelerationX, accelerationY, accelerationZ,
                        angularRateX, angularRateY, angularRateZ);

        final ECIFrame result19 = ECIInertialNavigator
                .navigateECIAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECIFrame result20 = ECIInertialNavigator
                .navigateECIAndReturnNew(timeInterval,
                        oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECIFrame result21 = ECIInertialNavigator
                .navigateECIAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECIFrame result22 = ECIInertialNavigator
                .navigateECIAndReturnNew(timeInterval,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECIFrame result23 = ECIInertialNavigator
                .navigateECIAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ,
                        accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECIFrame result24 = ECIInertialNavigator
                .navigateECIAndReturnNew(timeInterval,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ,
                        accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECIFrame result25 = ECIInertialNavigator
                .navigateECIAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                        accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECIFrame result26 = ECIInertialNavigator
                .navigateECIAndReturnNew(timeInterval,
                        oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                        accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECIFrame result27 = ECIInertialNavigator
                .navigateECIAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final ECIFrame result28 = ECIInertialNavigator
                .navigateECIAndReturnNew(timeInterval,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final ECIFrame result29 = ECIInertialNavigator
                .navigateECIAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldFrame, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECIFrame result30 = ECIInertialNavigator
                .navigateECIAndReturnNew(timeInterval,
                        oldFrame, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECIFrame result31 = ECIInertialNavigator
                .navigateECIAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldFrame, kinematics);

        final ECIFrame result32 = ECIInertialNavigator
                .navigateECIAndReturnNew(timeInterval,
                        oldFrame, kinematics);

        final ECIFrame result33 = ECIInertialNavigator
                .navigateECIAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldFrame, accelerationX, accelerationY, accelerationZ,
                        angularRateX, angularRateY, angularRateZ);

        final ECIFrame result34 = ECIInertialNavigator
                .navigateECIAndReturnNew(timeInterval,
                        oldFrame, accelerationX, accelerationY, accelerationZ,
                        angularRateX, angularRateY, angularRateZ);

        final ECIFrame result35 = ECIInertialNavigator
                .navigateECIAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldFrame, fx, fy, fz,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECIFrame result36 = ECIInertialNavigator
                .navigateECIAndReturnNew(timeInterval,
                        oldFrame, fx, fy, fz,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECIFrame result37 = ECIInertialNavigator
                .navigateECIAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldFrame, accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECIFrame result38 = ECIInertialNavigator
                .navigateECIAndReturnNew(timeInterval,
                        oldFrame, accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        assertEquals(result1, result2);
        assertEquals(result1, result3);
        assertEquals(result1, result4);
        assertEquals(result1, result5);
        assertEquals(result1, result6);
        assertEquals(result1, result7);
        assertEquals(result1, result8);
        assertEquals(result1, result9);
        assertEquals(result1, result10);
        assertEquals(result1, result11);
        assertEquals(result1, result12);
        assertEquals(result1, result13);
        assertEquals(result1, result14);
        assertEquals(result1, result15);
        assertEquals(result1, result16);
        assertEquals(result1, result17);
        assertEquals(result1, result18);
        assertEquals(result1, result19);
        assertEquals(result1, result20);
        assertEquals(result1, result21);
        assertEquals(result1, result22);
        assertEquals(result1, result23);
        assertEquals(result1, result24);
        assertEquals(result1, result25);
        assertEquals(result1, result26);
        assertEquals(result1, result27);
        assertEquals(result1, result28);
        assertEquals(result1, result29);
        assertEquals(result1, result30);
        assertEquals(result1, result31);
        assertEquals(result1, result32);
        assertEquals(result1, result33);
        assertEquals(result1, result34);
        assertEquals(result1, result35);
        assertEquals(result1, result36);
        assertEquals(result1, result37);
        assertEquals(result1, result38);
    }

    @Test
    public void testCompareNavigations()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, WrongSizeException,
            InertialNavigatorException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double latitude = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

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

            final NEDFrame oldNedFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
            final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(oldNedFrame);
            final ECIFrame oldFrame = ECEFtoECIFrameConverter.convertECEFtoECIAndReturnNew(
                    TIME_INTERVAL_SECONDS, oldEcefFrame);
            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(oldEcefFrame);
            final Matrix g = gravity.asMatrix();
            final Matrix cbe = oldEcefFrame.getCoordinateTransformation().getMatrix();
            final Matrix f = cbe.multiplyAndReturnNew(g);

            final double fx = f.getElementAtIndex(0);
            final double fy = f.getElementAtIndex(1);
            final double fz = f.getElementAtIndex(2);

            final double angularRateXDegreesPerSecond = randomizer.nextDouble(
                    MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                    MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
            final double angularRateYDegreesPerSecond = randomizer.nextDouble(
                    MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                    MAX_ANGULAR_RATE_DEGREES_PER_SECOND);
            final double angularRateZDegreesPerSecond = randomizer.nextDouble(
                    MIN_ANGULAR_RATE_DEGREES_PER_SECOND,
                    MAX_ANGULAR_RATE_DEGREES_PER_SECOND);

            final AngularSpeed angularSpeedX = new AngularSpeed(angularRateXDegreesPerSecond,
                    AngularSpeedUnit.DEGREES_PER_SECOND);
            final AngularSpeed angularSpeedY = new AngularSpeed(angularRateYDegreesPerSecond,
                    AngularSpeedUnit.DEGREES_PER_SECOND);
            final AngularSpeed angularSpeedZ = new AngularSpeed(angularRateZDegreesPerSecond,
                    AngularSpeedUnit.DEGREES_PER_SECOND);

            final double angularRateX = AngularSpeedConverter.convert(
                    angularSpeedX.getValue().doubleValue(), angularSpeedX.getUnit(),
                    AngularSpeedUnit.RADIANS_PER_SECOND);
            final double angularRateY = AngularSpeedConverter.convert(
                    angularSpeedY.getValue().doubleValue(), angularSpeedY.getUnit(),
                    AngularSpeedUnit.RADIANS_PER_SECOND);
            final double angularRateZ = AngularSpeedConverter.convert(
                    angularSpeedZ.getValue().doubleValue(), angularSpeedZ.getUnit(),
                    AngularSpeedUnit.RADIANS_PER_SECOND);

            final BodyKinematics kinematics = new BodyKinematics(fx, fy, fz,
                    angularRateX, angularRateY, angularRateZ);

            final ECIFrame newFrame = ECIInertialNavigator.navigateECIAndReturnNew(
                    TIME_INTERVAL_SECONDS, oldFrame, kinematics);

            final ECEFFrame newEcefFrame1 = ECEFInertialNavigator.navigateECEFAndReturnNew(
                    TIME_INTERVAL_SECONDS, oldEcefFrame, kinematics);
            final ECEFFrame newEcefFrame2 = ECItoECEFFrameConverter.convertECItoECEFAndReturnNew(
                    TIME_INTERVAL_SECONDS, newFrame);

            final NEDFrame newNedFrame1 = ECEFtoNEDFrameConverter
                    .convertECEFtoNEDAndReturnNew(newEcefFrame1);
            final NEDFrame newNedFrame2 = ECEFtoNEDFrameConverter
                    .convertECEFtoNEDAndReturnNew(newEcefFrame2);

            // compare
            assertTrue(newNedFrame1.equals(newNedFrame2, ABSOLUTE_ERROR));

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }
}
