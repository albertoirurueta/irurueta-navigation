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
package com.irurueta.navigation.inertial.navigators;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.ECEFGravity;
import com.irurueta.navigation.inertial.ECEFPosition;
import com.irurueta.navigation.inertial.ECEFVelocity;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.*;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class ECEFInertialNavigatorTest {
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

    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-2;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-1;

    private static final int TIMES = 100;

    @Test(expected = InvalidSourceAndDestinationFrameTypeException.class)
    public void testNavigateEcefWhenInvalidCoordinateTransformationMatrix()
            throws InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {
        final CoordinateTransformation c = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        final ECEFFrame result = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(0.0,
                0.0, 0.0, 0.0, c,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, result);
    }

    @Test
    public void testNavigate() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {
        final ECEFInertialNavigator navigator = new ECEFInertialNavigator();

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
        final ECEFFrame oldFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

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

        final ECEFFrame result1 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result1);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final ECEFFrame result2 = new ECEFFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result2);

        final ECEFPosition oldEcefPosition = new ECEFPosition(oldX, oldY, oldZ);
        final ECEFFrame result3 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result3);

        final ECEFFrame result4 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result4);

        final ECEFVelocity oldEcefVelocity = new ECEFVelocity(oldVx, oldVy, oldVz);
        final ECEFFrame result5 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC,
                oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result5);

        final ECEFFrame result6 = new ECEFFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldEcefVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result6);

        final ECEFFrame result7 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result7);

        final ECEFFrame result8 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC, oldEcefVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result8);

        final ECEFFrame result9 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, kinematics, result9);

        final ECEFFrame result10 = new ECEFFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, kinematics, result10);

        final ECEFFrame result11 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldVx, oldVy, oldVz, kinematics, result11);

        final ECEFFrame result12 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                kinematics, result12);

        final ECEFFrame result13 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC,
                oldEcefVelocity, kinematics, result13);

        final ECEFFrame result14 = new ECEFFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC, oldEcefVelocity,
                kinematics, result14);

        final ECEFFrame result15 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldEcefVelocity, kinematics, result15);

        final ECEFFrame result16 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC, oldEcefVelocity,
                kinematics, result16);

        final Point3D oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);
        final ECEFFrame result17 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result17);

        final ECEFFrame result18 = new ECEFFrame();
        navigator.navigate(timeInterval, oldPosition,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result18);

        final ECEFFrame result19 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result19);

        final ECEFFrame result20 = new ECEFFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldEcefVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result20);

        final ECEFFrame result21 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition,
                oldC, oldVx, oldVy, oldVz, kinematics, result21);

        final ECEFFrame result22 = new ECEFFrame();
        navigator.navigate(timeInterval, oldPosition,
                oldC, oldVx, oldVy, oldVz, kinematics, result22);

        final ECEFFrame result23 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldEcefVelocity, kinematics, result23);

        final ECEFFrame result24 = new ECEFFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldEcefVelocity,
                kinematics, result24);

        final Distance oldDistanceX = new Distance(oldX, DistanceUnit.METER);
        final Distance oldDistanceY = new Distance(oldY, DistanceUnit.METER);
        final Distance oldDistanceZ = new Distance(oldZ, DistanceUnit.METER);
        final ECEFFrame result25 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result25);

        final ECEFFrame result26 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result26);

        final ECEFFrame result27 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ,
                oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result27);

        final ECEFFrame result28 = new ECEFFrame();
        navigator.navigate(timeInterval, oldDistanceX, oldDistanceY, oldDistanceZ,
                oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result28);

        final ECEFFrame result29 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics, result29);

        final ECEFFrame result30 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics, result30);

        final ECEFFrame result31 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldEcefVelocity, kinematics, result31);

        final ECEFFrame result32 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldEcefVelocity, kinematics, result32);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);
        final ECEFFrame result33 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result33);

        final ECEFFrame result34 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result34);

        final ECEFFrame result35 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result35);

        final ECEFFrame result36 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result36);

        final ECEFFrame result37 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result37);

        final ECEFFrame result38 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result38);

        final ECEFFrame result39 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition,
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ, kinematics,
                result39);

        final ECEFFrame result40 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition,
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ, kinematics,
                result40);

        final Acceleration accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final ECEFFrame result41 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result41);

        final ECEFFrame result42 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result42);

        final ECEFFrame result43 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition,
                oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result43);

        final ECEFFrame result44 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition,
                oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result44);

        final ECEFFrame result45 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                oldC, oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result45);

        final ECEFFrame result46 = new ECEFFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ,
                oldC, oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result46);

        final ECEFFrame result47 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result47);

        final ECEFFrame result48 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC,
                oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result48);

        final ECEFFrame result49 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result49);

        final ECEFFrame result50 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result50);

        final ECEFFrame result51 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result51);

        final ECEFFrame result52 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result52);

        final ECEFFrame result53 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC,
                oldEcefVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result53);

        final ECEFFrame result54 = new ECEFFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ, oldC,
                oldEcefVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result54);

        final ECEFFrame result55 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldEcefVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result55);

        final ECEFFrame result56 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC, oldEcefVelocity,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ,
                result56);

        final ECEFFrame result57 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result57);

        final ECEFFrame result58 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result58);

        final ECEFFrame result59 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result59);

        final ECEFFrame result60 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result60);

        final ECEFFrame result61 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result61);

        final ECEFFrame result62 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result62);

        final ECEFFrame result63 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result63);

        final ECEFFrame result64 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result64);

        final ECEFFrame result65 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result65);

        final ECEFFrame result66 = new ECEFFrame();
        navigator.navigate(timeInterval, oldEcefPosition, oldC,
                oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result66);

        final ECEFFrame result67 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result67);

        final ECEFFrame result68 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result68);

        final ECEFFrame result69 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result69);

        final ECEFFrame result70 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result70);

        final ECEFFrame result71 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result71);

        final ECEFFrame result72 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldX, oldY, oldZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result72);

        final ECEFFrame result73 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics,
                result73);

        final ECEFFrame result74 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics,
                result74);

        final ECEFFrame result75 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result75);

        final ECEFFrame result76 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result76);

        final ECEFFrame result77 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldFrame, kinematics, result77);

        final ECEFFrame result78 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldFrame, kinematics, result78);

        final ECEFFrame result79 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result79);

        final ECEFFrame result80 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result80);

        final ECEFFrame result81 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result81);

        final ECEFFrame result82 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result82);

        final ECEFFrame result83 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result83);

        final ECEFFrame result84 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result84);

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
        assertEquals(result1, result39);
        assertEquals(result1, result40);
        assertEquals(result1, result41);
        assertEquals(result1, result42);
        assertEquals(result1, result43);
        assertEquals(result1, result44);
        assertEquals(result1, result45);
        assertEquals(result1, result46);
        assertEquals(result1, result47);
        assertEquals(result1, result48);
        assertEquals(result1, result49);
        assertEquals(result1, result50);
        assertEquals(result1, result51);
        assertEquals(result1, result52);
        assertEquals(result1, result53);
        assertEquals(result1, result54);
        assertEquals(result1, result55);
        assertEquals(result1, result56);
        assertEquals(result1, result57);
        assertEquals(result1, result58);
        assertEquals(result1, result59);
        assertEquals(result1, result60);
        assertEquals(result1, result61);
        assertEquals(result1, result62);
        assertEquals(result1, result63);
        assertEquals(result1, result64);
        assertEquals(result1, result65);
        assertEquals(result1, result66);
        assertEquals(result1, result67);
        assertEquals(result1, result68);
        assertEquals(result1, result69);
        assertEquals(result1, result70);
        assertEquals(result1, result71);
        assertEquals(result1, result72);
        assertEquals(result1, result73);
        assertEquals(result1, result74);
        assertEquals(result1, result75);
        assertEquals(result1, result76);
        assertEquals(result1, result77);
        assertEquals(result1, result78);
        assertEquals(result1, result79);
        assertEquals(result1, result80);
        assertEquals(result1, result81);
        assertEquals(result1, result82);
        assertEquals(result1, result83);
        assertEquals(result1, result84);
    }

    @Test
    public void testNavigateAndReturnNew() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {
        final ECEFInertialNavigator navigator = new ECEFInertialNavigator();

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
        final ECEFFrame oldFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

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

        final ECEFFrame result1 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final ECEFFrame result2 = navigator.navigateAndReturnNew(
                timeInterval, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFPosition oldEcefPosition = new ECEFPosition(oldX, oldY, oldZ);
        final ECEFFrame result3 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result4 = navigator.navigateAndReturnNew(timeInterval,
                oldEcefPosition, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFVelocity oldEcefVelocity = new ECEFVelocity(oldVx, oldVy, oldVz);
        final ECEFFrame result5 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldEcefVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result6 = navigator.navigateAndReturnNew(timeInterval,
                oldX, oldY, oldZ, oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result7 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldEcefVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result8 = navigator.navigateAndReturnNew(timeInterval,
                oldEcefPosition, oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result9 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics);

        final ECEFFrame result10 = navigator.navigateAndReturnNew(
                timeInterval, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, kinematics);

        final ECEFFrame result11 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                kinematics);

        final ECEFFrame result12 = navigator.navigateAndReturnNew(timeInterval,
                oldEcefPosition, oldC, oldVx, oldVy, oldVz, kinematics);

        final ECEFFrame result13 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldEcefVelocity,
                kinematics);

        final ECEFFrame result14 = navigator.navigateAndReturnNew(timeInterval,
                oldX, oldY, oldZ, oldC, oldEcefVelocity, kinematics);

        final ECEFFrame result15 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldEcefVelocity,
                kinematics);

        final ECEFFrame result16 = navigator.navigateAndReturnNew(timeInterval,
                oldEcefPosition, oldC, oldEcefVelocity, kinematics);

        final Point3D oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);
        final ECEFFrame result17 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result18 = navigator.navigateAndReturnNew(
                timeInterval, oldPosition,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result19 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC, oldEcefVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result20 = navigator.navigateAndReturnNew(timeInterval,
                oldPosition, oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result21 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition,
                oldC, oldVx, oldVy, oldVz, kinematics);

        final ECEFFrame result22 = navigator.navigateAndReturnNew(
                timeInterval, oldPosition,
                oldC, oldVx, oldVy, oldVz, kinematics);

        final ECEFFrame result23 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC, oldEcefVelocity,
                kinematics);

        final ECEFFrame result24 = navigator.navigateAndReturnNew(timeInterval,
                oldPosition, oldC, oldEcefVelocity, kinematics);

        final Distance oldDistanceX = new Distance(oldX, DistanceUnit.METER);
        final Distance oldDistanceY = new Distance(oldY, DistanceUnit.METER);
        final Distance oldDistanceZ = new Distance(oldZ, DistanceUnit.METER);
        final ECEFFrame result25 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result26 = navigator.navigateAndReturnNew(
                timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result27 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ,
                oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result28 = navigator.navigateAndReturnNew(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result29 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics);

        final ECEFFrame result30 = navigator.navigateAndReturnNew(
                timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics);

        final ECEFFrame result31 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ,
                oldC, oldEcefVelocity, kinematics);

        final ECEFFrame result32 = navigator.navigateAndReturnNew(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity,
                kinematics);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);
        final ECEFFrame result33 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result34 = navigator.navigateAndReturnNew(
                timeInterval,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result35 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result36 = navigator.navigateAndReturnNew(timeInterval,
                oldEcefPosition, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result37 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics);

        final ECEFFrame result38 = navigator.navigateAndReturnNew(
                timeInterval,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics);

        final ECEFFrame result39 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final ECEFFrame result40 = navigator.navigateAndReturnNew(timeInterval,
                oldEcefPosition, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics);

        final Acceleration accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final ECEFFrame result41 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result42 = navigator.navigateAndReturnNew(
                timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result43 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result44 = navigator.navigateAndReturnNew(timeInterval,
                oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result45 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result46 = navigator.navigateAndReturnNew(timeInterval,
                oldX, oldY, oldZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result47 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result48 = navigator.navigateAndReturnNew(timeInterval,
                oldEcefPosition, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result49 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result50 = navigator.navigateAndReturnNew(
                timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result51 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result52 = navigator.navigateAndReturnNew(timeInterval,
                oldEcefPosition, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result53 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC, oldEcefVelocity,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result54 = navigator.navigateAndReturnNew(timeInterval,
                oldX, oldY, oldZ, oldC, oldEcefVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result55 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldEcefVelocity,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result56 = navigator.navigateAndReturnNew(timeInterval,
                oldEcefPosition, oldC, oldEcefVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result57 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result58 = navigator.navigateAndReturnNew(
                timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result59 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result60 = navigator.navigateAndReturnNew(
                timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result61 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldEcefPosition, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result62 = navigator.navigateAndReturnNew(timeInterval,
                oldEcefPosition, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result63 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result64 = navigator.navigateAndReturnNew(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result65 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result66 = navigator.navigateAndReturnNew(timeInterval,
                oldEcefPosition, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result67 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result68 = navigator.navigateAndReturnNew(
                timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result69 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result70 = navigator.navigateAndReturnNew(timeInterval,
                oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result71 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldX, oldY, oldZ, oldC,
                oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result72 = navigator.navigateAndReturnNew(timeInterval,
                oldX, oldY, oldZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result73 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final ECEFFrame result74 = navigator.navigateAndReturnNew(
                timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final ECEFFrame result75 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result76 = navigator.navigateAndReturnNew(
                timeInterval,
                oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result77 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, kinematics);

        final ECEFFrame result78 = navigator.navigateAndReturnNew(
                timeInterval, oldFrame, kinematics);

        final ECEFFrame result79 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result80 = navigator.navigateAndReturnNew(
                timeInterval,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result81 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result82 = navigator.navigateAndReturnNew(
                timeInterval,
                oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result83 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result84 = navigator.navigateAndReturnNew(
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
        assertEquals(result1, result39);
        assertEquals(result1, result40);
        assertEquals(result1, result41);
        assertEquals(result1, result42);
        assertEquals(result1, result43);
        assertEquals(result1, result44);
        assertEquals(result1, result45);
        assertEquals(result1, result46);
        assertEquals(result1, result47);
        assertEquals(result1, result48);
        assertEquals(result1, result49);
        assertEquals(result1, result50);
        assertEquals(result1, result51);
        assertEquals(result1, result52);
        assertEquals(result1, result53);
        assertEquals(result1, result54);
        assertEquals(result1, result55);
        assertEquals(result1, result56);
        assertEquals(result1, result57);
        assertEquals(result1, result58);
        assertEquals(result1, result59);
        assertEquals(result1, result60);
        assertEquals(result1, result61);
        assertEquals(result1, result62);
        assertEquals(result1, result63);
        assertEquals(result1, result64);
        assertEquals(result1, result65);
        assertEquals(result1, result66);
        assertEquals(result1, result67);
        assertEquals(result1, result68);
        assertEquals(result1, result69);
        assertEquals(result1, result70);
        assertEquals(result1, result71);
        assertEquals(result1, result72);
        assertEquals(result1, result73);
        assertEquals(result1, result74);
        assertEquals(result1, result75);
        assertEquals(result1, result76);
        assertEquals(result1, result77);
        assertEquals(result1, result78);
        assertEquals(result1, result79);
        assertEquals(result1, result80);
        assertEquals(result1, result81);
        assertEquals(result1, result82);
        assertEquals(result1, result83);
        assertEquals(result1, result84);
    }

    @Test
    public void testNavigateEcef() throws InvalidRotationMatrixException,
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
        final ECEFFrame oldFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

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

        final ECEFFrame result1 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result1);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final ECEFFrame result2 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result2);

        final ECEFPosition oldEcefPosition = new ECEFPosition(oldX, oldY, oldZ);
        final ECEFFrame result3 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldEcefPosition,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result3);

        final ECEFFrame result4 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result4);

        final ECEFVelocity oldEcefVelocity = new ECEFVelocity(oldVx, oldVy, oldVz);
        final ECEFFrame result5 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result5);

        final ECEFFrame result6 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ, oldC,
                oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result6);

        final ECEFFrame result7 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldEcefPosition,
                oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result7);

        final ECEFFrame result8 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC,
                oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result8);

        final ECEFFrame result9 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, kinematics, result9);

        final ECEFFrame result10 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, kinematics, result10);

        final ECEFFrame result11 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldEcefPosition,
                oldC, oldVx, oldVy, oldVz, kinematics, result11);

        final ECEFFrame result12 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC,
                oldVx, oldVy, oldVz, kinematics, result12);

        final ECEFFrame result13 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                oldC, oldEcefVelocity, kinematics, result13);

        final ECEFFrame result14 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ, oldC,
                oldEcefVelocity, kinematics, result14);

        final ECEFFrame result15 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldEcefPosition,
                oldC, oldEcefVelocity, kinematics, result15);

        final ECEFFrame result16 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC,
                oldEcefVelocity, kinematics, result16);

        final Point3D oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);
        final ECEFFrame result17 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldPosition,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result17);

        final ECEFFrame result18 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldPosition,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result18);

        final ECEFFrame result19 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldPosition,
                oldC, oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result19);

        final ECEFFrame result20 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldPosition, oldC,
                oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result20);

        final ECEFFrame result21 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldPosition,
                oldC, oldVx, oldVy, oldVz, kinematics, result21);

        final ECEFFrame result22 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldPosition,
                oldC, oldVx, oldVy, oldVz, kinematics, result22);

        final ECEFFrame result23 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldPosition,
                oldC, oldEcefVelocity, kinematics, result23);

        final ECEFFrame result24 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldPosition,
                oldC, oldEcefVelocity, kinematics, result24);

        final Distance oldDistanceX = new Distance(oldX, DistanceUnit.METER);
        final Distance oldDistanceY = new Distance(oldY, DistanceUnit.METER);
        final Distance oldDistanceZ = new Distance(oldZ, DistanceUnit.METER);
        final ECEFFrame result25 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result25);

        final ECEFFrame result26 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result26);

        final ECEFFrame result27 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result27);

        final ECEFFrame result28 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldEcefVelocity, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result28);

        final ECEFFrame result29 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics, result29);

        final ECEFFrame result30 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics, result30);

        final ECEFFrame result31 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldEcefVelocity, kinematics, result31);

        final ECEFFrame result32 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldEcefVelocity, kinematics, result32);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);
        final ECEFFrame result33 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result33);

        final ECEFFrame result34 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result34);

        final ECEFFrame result35 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldEcefPosition, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result35);

        final ECEFFrame result36 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval,
                oldEcefPosition, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result36);

        final ECEFFrame result37 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result37);

        final ECEFFrame result38 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result38);

        final ECEFFrame result39 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldEcefPosition, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result39);

        final ECEFFrame result40 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition,
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result40);

        final Acceleration accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final ECEFFrame result41 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result41);

        final ECEFFrame result42 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result42);

        final ECEFFrame result43 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result43);

        final ECEFFrame result44 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition,
                oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result44);

        final ECEFFrame result45 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                oldC, oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result45);

        final ECEFFrame result46 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ, oldC,
                oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result46);

        final ECEFFrame result47 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldEcefPosition, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result47);

        final ECEFFrame result48 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC,
                oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result48);

        final ECEFFrame result49 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result49);

        final ECEFFrame result50 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result50);

        final ECEFFrame result51 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldEcefPosition, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result51);

        final ECEFFrame result52 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval,
                oldEcefPosition, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result52);

        final ECEFFrame result53 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldEcefVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result53);

        final ECEFFrame result54 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ, oldC,
                oldEcefVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result54);

        final ECEFFrame result55 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldEcefPosition,
                oldC, oldEcefVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result55);

        final ECEFFrame result56 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition,
                oldC, oldEcefVelocity, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result56);

        final ECEFFrame result57 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result57);

        final ECEFFrame result58 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result58);

        final ECEFFrame result59 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result59);

        final ECEFFrame result60 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result60);

        final ECEFFrame result61 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldEcefPosition, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result61);

        final ECEFFrame result62 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result62);

        final ECEFFrame result63 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result63);

        final ECEFFrame result64 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC, oldEcefVelocity,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result64);

        final ECEFFrame result65 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldEcefPosition,
                oldC, oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result65);

        final ECEFFrame result66 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC,
                oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result66);

        final ECEFFrame result67 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result67);

        final ECEFFrame result68 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result68);

        final ECEFFrame result69 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result69);

        final ECEFFrame result70 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldEcefPosition, oldC,
                oldVx, oldVy, oldVz, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result70);

        final ECEFFrame result71 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                oldC, oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result71);

        final ECEFFrame result72 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ, oldC,
                oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result72);

        final ECEFFrame result73 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics,
                result73);

        final ECEFFrame result74 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics,
                result74);

        final ECEFFrame result75 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result75);

        final ECEFFrame result76 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval,
                oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result76);

        final ECEFFrame result77 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldFrame, kinematics, result77);

        final ECEFFrame result78 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval,
                oldFrame, kinematics, result78);

        final ECEFFrame result79 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result79);

        final ECEFFrame result80 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result80);

        final ECEFFrame result81 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result81);

        final ECEFFrame result82 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval,
                oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result82);

        final ECEFFrame result83 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result83);

        final ECEFFrame result84 = new ECEFFrame();
        ECEFInertialNavigator.navigateECEF(timeInterval,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result84);

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
        assertEquals(result1, result39);
        assertEquals(result1, result40);
        assertEquals(result1, result41);
        assertEquals(result1, result42);
        assertEquals(result1, result43);
        assertEquals(result1, result44);
        assertEquals(result1, result45);
        assertEquals(result1, result46);
        assertEquals(result1, result47);
        assertEquals(result1, result48);
        assertEquals(result1, result49);
        assertEquals(result1, result50);
        assertEquals(result1, result51);
        assertEquals(result1, result52);
        assertEquals(result1, result53);
        assertEquals(result1, result54);
        assertEquals(result1, result55);
        assertEquals(result1, result56);
        assertEquals(result1, result57);
        assertEquals(result1, result58);
        assertEquals(result1, result59);
        assertEquals(result1, result60);
        assertEquals(result1, result61);
        assertEquals(result1, result62);
        assertEquals(result1, result63);
        assertEquals(result1, result64);
        assertEquals(result1, result65);
        assertEquals(result1, result66);
        assertEquals(result1, result67);
        assertEquals(result1, result68);
        assertEquals(result1, result69);
        assertEquals(result1, result70);
        assertEquals(result1, result71);
        assertEquals(result1, result72);
        assertEquals(result1, result73);
        assertEquals(result1, result74);
        assertEquals(result1, result75);
        assertEquals(result1, result76);
        assertEquals(result1, result77);
        assertEquals(result1, result78);
        assertEquals(result1, result79);
        assertEquals(result1, result80);
        assertEquals(result1, result81);
        assertEquals(result1, result82);
        assertEquals(result1, result83);
        assertEquals(result1, result84);
    }

    @Test
    public void testNavigateEcefAndReturnNew() throws InvalidRotationMatrixException,
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
        final ECEFFrame oldFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

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

        final ECEFFrame result1 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                        oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final ECEFFrame result2 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ,
                        oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFPosition oldEcefPosition = new ECEFPosition(oldX, oldY, oldZ);
        final ECEFFrame result3 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition,
                        oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result4 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                        oldVx, oldVy, oldVz, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFVelocity oldEcefVelocity = new ECEFVelocity(oldVx, oldVy, oldVz);
        final ECEFFrame result5 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                        oldC, oldEcefVelocity, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result6 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                        oldEcefVelocity, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result7 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition,
                        oldC, oldEcefVelocity, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result8 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                        oldEcefVelocity, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result9 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                        oldC, oldVx, oldVy, oldVz, kinematics);

        final ECEFFrame result10 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ,
                        oldC, oldVx, oldVy, oldVz, kinematics);

        final ECEFFrame result11 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition,
                        oldC, oldVx, oldVy, oldVz, kinematics);

        final ECEFFrame result12 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                        oldVx, oldVy, oldVz, kinematics);

        final ECEFFrame result13 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                        oldC, oldEcefVelocity, kinematics);

        final ECEFFrame result14 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                        oldEcefVelocity, kinematics);

        final ECEFFrame result15 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition,
                        oldC, oldEcefVelocity, kinematics);

        final ECEFFrame result16 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                        oldEcefVelocity, kinematics);

        final Point3D oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);
        final ECEFFrame result17 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition,
                        oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result18 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldPosition,
                        oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result19 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition,
                        oldC, oldEcefVelocity, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result20 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldPosition, oldC,
                        oldEcefVelocity, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result21 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition,
                        oldC, oldVx, oldVy, oldVz, kinematics);

        final ECEFFrame result22 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldPosition,
                        oldC, oldVx, oldVy, oldVz, kinematics);

        final ECEFFrame result23 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition,
                        oldC, oldEcefVelocity, kinematics);

        final ECEFFrame result24 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldPosition, oldC,
                        oldEcefVelocity, kinematics);

        final Distance oldDistanceX = new Distance(oldX, DistanceUnit.METER);
        final Distance oldDistanceY = new Distance(oldY, DistanceUnit.METER);
        final Distance oldDistanceZ = new Distance(oldZ, DistanceUnit.METER);
        final ECEFFrame result25 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldVx, oldVy, oldVz, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result26 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldVx, oldVy, oldVz, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result27 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldEcefVelocity, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result28 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldEcefVelocity, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result29 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldVx, oldVy, oldVz, kinematics);

        final ECEFFrame result30 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldVx, oldVy, oldVz, kinematics);

        final ECEFFrame result31 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldEcefVelocity, kinematics);

        final ECEFFrame result32 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldEcefVelocity, kinematics);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);
        final ECEFFrame result33 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                        fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result34 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                        fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result35 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldEcefPosition, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                        fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result36 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result37 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                        kinematics);

        final ECEFFrame result38 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                        kinematics);

        final ECEFFrame result39 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition,
                        oldC, oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final ECEFFrame result40 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final Acceleration accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final ECEFFrame result41 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                        accelerationX, accelerationY, accelerationZ,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result42 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                        accelerationX, accelerationY, accelerationZ,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result43 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition,
                        oldC, oldVx, oldVy, oldVz,
                        accelerationX, accelerationY, accelerationZ,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result44 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                        oldVx, oldVy, oldVz,
                        accelerationX, accelerationY, accelerationZ,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result45 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                        oldC, oldEcefVelocity,
                        accelerationX, accelerationY, accelerationZ,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result46 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                        oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result47 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition,
                        oldC, oldEcefVelocity,
                        accelerationX, accelerationY, accelerationZ,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result48 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                        oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result49 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result50 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result51 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition,
                        oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result52 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                        oldVx, oldVy, oldVz, fx, fy, fz,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result53 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                        oldC, oldEcefVelocity, fx, fy, fz,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result54 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                        oldEcefVelocity, fx, fy, fz,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result55 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition,
                        oldC, oldEcefVelocity, fx, fy, fz,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result56 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                        oldEcefVelocity, fx, fy, fz,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result57 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result58 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result59 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ,
                        accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result60 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ,
                        accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result61 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition,
                        oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                        accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result62 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ,
                        accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result63 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result64 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result65 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefPosition,
                        oldC, oldEcefVelocity,
                        accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result66 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                        oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result67 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                        accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result68 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                        accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result69 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldEcefPosition, oldC, oldVx, oldVy, oldVz,
                        accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result70 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldEcefPosition, oldC,
                        oldVx, oldVy, oldVz,
                        accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result71 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                        oldC, oldEcefVelocity,
                        accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result72 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                        oldEcefVelocity, accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result73 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final ECEFFrame result74 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final ECEFFrame result75 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldFrame, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result76 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldFrame, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result77 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldFrame, kinematics);

        final ECEFFrame result78 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldFrame, kinematics);

        final ECEFFrame result79 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldFrame, accelerationX, accelerationY, accelerationZ,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result80 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldFrame, accelerationX, accelerationY, accelerationZ,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result81 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldFrame, fx, fy, fz,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result82 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldFrame, fx, fy, fz,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result83 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldFrame, accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result84 = ECEFInertialNavigator
                .navigateECEFAndReturnNew(timeInterval,
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
        assertEquals(result1, result39);
        assertEquals(result1, result40);
        assertEquals(result1, result41);
        assertEquals(result1, result42);
        assertEquals(result1, result43);
        assertEquals(result1, result44);
        assertEquals(result1, result45);
        assertEquals(result1, result46);
        assertEquals(result1, result47);
        assertEquals(result1, result48);
        assertEquals(result1, result49);
        assertEquals(result1, result50);
        assertEquals(result1, result51);
        assertEquals(result1, result52);
        assertEquals(result1, result53);
        assertEquals(result1, result54);
        assertEquals(result1, result55);
        assertEquals(result1, result56);
        assertEquals(result1, result57);
        assertEquals(result1, result58);
        assertEquals(result1, result59);
        assertEquals(result1, result60);
        assertEquals(result1, result61);
        assertEquals(result1, result62);
        assertEquals(result1, result63);
        assertEquals(result1, result64);
        assertEquals(result1, result65);
        assertEquals(result1, result66);
        assertEquals(result1, result67);
        assertEquals(result1, result68);
        assertEquals(result1, result69);
        assertEquals(result1, result70);
        assertEquals(result1, result71);
        assertEquals(result1, result72);
        assertEquals(result1, result73);
        assertEquals(result1, result74);
        assertEquals(result1, result75);
        assertEquals(result1, result76);
        assertEquals(result1, result77);
        assertEquals(result1, result78);
        assertEquals(result1, result79);
        assertEquals(result1, result80);
        assertEquals(result1, result81);
        assertEquals(result1, result82);
        assertEquals(result1, result83);
        assertEquals(result1, result84);
    }

    @Test
    public void testNavigateFreeFallingBody() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException,
            WrongSizeException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double latitude = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final double vn = 0.0;
            final double ve = 0.0;
            final double vd = 0.0;

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
            final ECEFFrame oldFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(oldFrame);
            final Matrix g = gravity.asMatrix();
            final Matrix cbe = oldFrame.getCoordinateTransformationMatrix();
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

            final ECEFFrame newFrame = ECEFInertialNavigator.navigateECEFAndReturnNew(
                    TIME_INTERVAL_SECONDS,
                    oldFrame, kinematics);

            final NEDFrame newNedFrame = ECEFtoNEDFrameConverter
                    .convertECEFtoNEDAndReturnNew(newFrame);

            // Because body is in free fall its latitude and longitude does not change
            assertEquals(oldNedFrame.getLatitude(), newNedFrame.getLatitude(),
                    ABSOLUTE_ERROR);
            assertEquals(oldNedFrame.getLongitude(), newNedFrame.getLongitude(),
                    ABSOLUTE_ERROR);

            // Since body is falling, new height is smaller than the initial one
            assertTrue(newNedFrame.getHeight() < oldNedFrame.getHeight());

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testNavigateZeroTimeInterval() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException,
            WrongSizeException {

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
            final ECEFFrame oldFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);
            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(oldFrame);
            final Matrix g = gravity.asMatrix();
            final Matrix cbe = oldFrame.getCoordinateTransformation().getMatrix();
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

            final ECEFFrame newFrame = ECEFInertialNavigator.navigateECEFAndReturnNew(
                    0.0, oldFrame, kinematics);

            final NEDFrame newNedFrame = ECEFtoNEDFrameConverter
                    .convertECEFtoNEDAndReturnNew(newFrame);

            // Because time interval is zero, body hasn't moved
            assertEquals(oldNedFrame.getLatitude(), newNedFrame.getLatitude(),
                    ABSOLUTE_ERROR);
            assertEquals(oldNedFrame.getLongitude(), newNedFrame.getLongitude(),
                    ABSOLUTE_ERROR);
            assertEquals(oldNedFrame.getHeight(), newNedFrame.getHeight(),
                    ABSOLUTE_ERROR);
            assertTrue(oldNedFrame.equals(newNedFrame, ABSOLUTE_ERROR));

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testNavigateWithInitialVelocityAndNoSpecificForce() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {

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
            final ECEFFrame oldFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

            final double fx = 0.0;
            final double fy = 0.0;
            final double fz = 0.0;

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

            final ECEFFrame newFrame = ECEFInertialNavigator.navigateECEFAndReturnNew(
                    TIME_INTERVAL_SECONDS,
                    oldFrame, kinematics);

            final NEDFrame newNedFrame = ECEFtoNEDFrameConverter
                    .convertECEFtoNEDAndReturnNew(newFrame);

            // Because no specific force is applied to body, it will keep its inertia
            // (initial speed). This is approximate, since as the body moves the amount
            // of gravity applied to it changes as well
            final double oldSpeed = oldNedFrame.getVelocityNorm();
            double newSpeed = newNedFrame.getVelocityNorm();

            final Point3D oldPosition = oldFrame.getPosition();
            final Point3D newPosition = newFrame.getPosition();
            final double distance = oldPosition.distanceTo(newPosition);

            final double estimatedSpeed = distance / TIME_INTERVAL_SECONDS;

            assertEquals(estimatedSpeed, newSpeed, VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(oldSpeed, newSpeed, 2.0 * VERY_LARGE_ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testNavigateWithNoInitialVelocityAndNoSpecificForce() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            final UniformRandomizer randomizer = new UniformRandomizer(new Random());

            final double latitude = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double longitude = Math.toRadians(
                    randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
            final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

            final double vn = 0.0;
            final double ve = 0.0;
            final double vd = 0.0;

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
            final ECEFFrame oldFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldNedFrame);

            final double fx = 0.0;
            final double fy = 0.0;
            final double fz = 0.0;

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

            final ECEFFrame newFrame = ECEFInertialNavigator.navigateECEFAndReturnNew(
                    TIME_INTERVAL_SECONDS, oldFrame, kinematics);

            final NEDFrame newNedFrame = ECEFtoNEDFrameConverter
                    .convertECEFtoNEDAndReturnNew(newFrame);

            // Because no forces are applied to the body and the body has no initial velocity,
            // the body will remain on the same position
            assertEquals(oldNedFrame.getLatitude(), newNedFrame.getLatitude(),
                    ABSOLUTE_ERROR);
            assertEquals(oldNedFrame.getLongitude(), newNedFrame.getLongitude(),
                    ABSOLUTE_ERROR);
            assertEquals(oldNedFrame.getHeight(), newNedFrame.getHeight(),
                    LARGE_ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }
}
