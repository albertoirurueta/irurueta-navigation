package com.irurueta.navigation.inertial.navigators;

import com.irurueta.algebra.Matrix;
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
import com.irurueta.navigation.inertial.NEDGravity;
import com.irurueta.navigation.inertial.NEDPosition;
import com.irurueta.navigation.inertial.NEDVelocity;
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.*;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class NEDInertialNavigatorTest {
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
    private static final double LARGE_ABSOLUTE_ERROR = 1e-3;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-1;

    private static final int TIMES = 100;

    @Test(expected = InvalidSourceAndDestinationFrameTypeException.class)
    public void testNavigateNEDWhenInvalidCoordinateTransformationMatrix()
            throws InvalidSourceAndDestinationFrameTypeException,
            InertialNavigatorException {

        final CoordinateTransformation c = new CoordinateTransformation(
                FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        final NEDFrame result = new NEDFrame();
        NEDInertialNavigator.navigateNED(0.0,
                0.0, 0.0, 0.0, c,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, result);
    }

    @Test
    public void testNavigate() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {

        final NEDInertialNavigator navigator = new NEDInertialNavigator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double oldLatitude = Math.toRadians(LATITUDE_DEGREES);
        final double oldLongitude = Math.toRadians(LONGITUDE_DEGREES);

        final double oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Quaternion q = new Quaternion(roll, pitch, yaw);

        final Matrix m = q.asInhomogeneousMatrix();
        final CoordinateTransformation oldC = new CoordinateTransformation(
                m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame oldFrame = new NEDFrame(oldLatitude, oldLongitude, HEIGHT,
                oldVn, oldVe, oldVd, oldC);

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

        final NEDFrame result1 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result1);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final NEDFrame result2 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result2);

        final NEDPosition oldPosition = new NEDPosition(oldLatitude, oldLongitude, HEIGHT);
        final NEDFrame result3 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVn, oldVe, oldVd,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result3);

        final NEDFrame result4 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result4);

        final NEDVelocity oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final NEDFrame result5 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result5);

        final NEDFrame result6 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result6);

        final NEDFrame result7 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result7);

        final NEDFrame result8 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result8);

        final NEDFrame result9 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, kinematics, result9);

        final NEDFrame result10 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, kinematics, result10);

        final NEDFrame result11 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVn, oldVe, oldVd, kinematics, result11);

        final NEDFrame result12 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC,
                oldVn, oldVe, oldVd, kinematics, result12);

        final NEDFrame result13 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVelocity, kinematics, result13);

        final NEDFrame result14 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVelocity, kinematics, result14);

        final NEDFrame result15 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                kinematics, result15);

        final NEDFrame result16 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVelocity,
                kinematics, result16);

        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final Angle oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final Distance oldHeightDistance = new Distance(HEIGHT, DistanceUnit.METER);
        final NEDFrame result17 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result17);

        final NEDFrame result18 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result18);

        final NEDFrame result19 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, fx, fy, fz, angularRateX,
                angularRateY, angularRateZ, result19);

        final NEDFrame result20 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, fx, fy, fz, angularRateX,
                angularRateY, angularRateZ, result20);

        final NEDFrame result21 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, kinematics, result21);

        final NEDFrame result22 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, kinematics,
                result22);

        final NEDFrame result23 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, kinematics, result23);

        final NEDFrame result24 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, kinematics, result24);

        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final NEDFrame result25 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result25);

        final NEDFrame result26 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result26);

        final NEDFrame result27 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result27);

        final NEDFrame result28 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result28);

        final NEDFrame result29 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics,
                result29);

        final NEDFrame result30 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics,
                result30);

        final NEDFrame result31 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, kinematics, result31);

        final NEDFrame result32 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, kinematics, result32);

        final Acceleration accelerationX = new Acceleration(fx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(fy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(fz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final NEDFrame result33 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result33);

        final NEDFrame result34 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result34);

        final NEDFrame result35 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result35);

        final NEDFrame result36 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result36);

        final NEDFrame result37 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result37);

        final NEDFrame result38 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result38);

        final NEDFrame result39 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result39);

        final NEDFrame result40 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX,
                angularRateY, angularRateZ, result40);

        final NEDFrame result41 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result41);

        final NEDFrame result42 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result42);

        final NEDFrame result43 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVn, oldVe,
                oldVd, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ,
                result43);

        final NEDFrame result44 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ,
                result44);

        final NEDFrame result45 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVelocity, fx, fy, fz, angularSpeedX,
                angularSpeedY, angularSpeedZ, result45);

        final NEDFrame result46 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, fx, fy, fz, angularSpeedX, angularSpeedY,
                angularSpeedZ, result46);

        final NEDFrame result47 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ,
                result47);

        final NEDFrame result48 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVelocity,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ,
                result48);

        final NEDFrame result49 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result49);

        final NEDFrame result50 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result50);

        final NEDFrame result51 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result51);

        final NEDFrame result52 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result52);

        final NEDFrame result53 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY,
                accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                result53);

        final NEDFrame result54 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY,
                accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                result54);

        final NEDFrame result55 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldSpeedN,
                oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result55);

        final NEDFrame result56 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldSpeedN,
                oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result56);

        final NEDFrame result57 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, accelerationX, accelerationY,
                accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result57);

        final NEDFrame result58 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, accelerationX, accelerationY,
                accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result58);

        final NEDFrame result59 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX,
                angularSpeedY, angularSpeedZ, result59);

        final NEDFrame result60 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX,
                angularSpeedY, angularSpeedZ, result60);

        final NEDFrame result61 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result61);

        final NEDFrame result62 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result62);

        final NEDFrame result63 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result63);

        final NEDFrame result64 = new NEDFrame();
        navigator.navigate(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularSpeedX,
                angularSpeedY, angularSpeedZ, result64);

        final NEDFrame result65 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result65);

        final NEDFrame result66 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result66);

        final NEDFrame result67 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                kinematics, result67);

        final NEDFrame result68 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                kinematics, result68);

        final NEDFrame result69 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result69);

        final NEDFrame result70 = new NEDFrame();
        navigator.navigate(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result70);

        final NEDFrame result71 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, kinematics,
                result71);

        final NEDFrame result72 = new NEDFrame();
        navigator.navigate(timeInterval, oldFrame, kinematics, result72);

        final NEDFrame result73 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result73);

        final NEDFrame result74 = new NEDFrame();
        navigator.navigate(timeInterval, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result74);

        final NEDFrame result75 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result75);

        final NEDFrame result76 = new NEDFrame();
        navigator.navigate(timeInterval, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result76);

        final NEDFrame result77 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result77);

        final NEDFrame result78 = new NEDFrame();
        navigator.navigate(timeInterval, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result78);

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
    }

    @Test
    public void testNavigateAndReturnNew() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {

        final NEDInertialNavigator navigator = new NEDInertialNavigator();

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double oldLatitude = Math.toRadians(LATITUDE_DEGREES);
        final double oldLongitude = Math.toRadians(LONGITUDE_DEGREES);

        final double oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Quaternion q = new Quaternion(roll, pitch, yaw);

        final Matrix m = q.asInhomogeneousMatrix();
        final CoordinateTransformation oldC = new CoordinateTransformation(
                m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame oldFrame = new NEDFrame(oldLatitude, oldLongitude, HEIGHT,
                oldVn, oldVe, oldVd, oldC);

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

        final NEDFrame result1 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final NEDFrame result2 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ);

        final NEDPosition oldPosition = new NEDPosition(oldLatitude, oldLongitude, HEIGHT);
        final NEDFrame result3 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVn, oldVe, oldVd,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final NEDFrame result4 = navigator.navigateAndReturnNew(
                timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final NEDVelocity oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final NEDFrame result5 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ);

        final NEDFrame result6 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ);

        final NEDFrame result7 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final NEDFrame result8 = navigator.navigateAndReturnNew(
                timeInterval, oldPosition, oldC, oldVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final NEDFrame result9 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, kinematics);

        final NEDFrame result10 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, kinematics);

        final NEDFrame result11 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVn, oldVe, oldVd, kinematics);

        final NEDFrame result12 = navigator.navigateAndReturnNew(
                timeInterval, oldPosition, oldC,
                oldVn, oldVe, oldVd, kinematics);

        final NEDFrame result13 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVelocity, kinematics);

        final NEDFrame result14 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVelocity, kinematics);

        final NEDFrame result15 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                kinematics);

        final NEDFrame result16 = navigator.navigateAndReturnNew(
                timeInterval, oldPosition, oldC, oldVelocity,
                kinematics);

        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final Angle oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final Distance oldHeightDistance = new Distance(HEIGHT, DistanceUnit.METER);
        final NEDFrame result17 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result18 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result19 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, fx, fy, fz, angularRateX,
                angularRateY, angularRateZ);

        final NEDFrame result20 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, fx, fy, fz, angularRateX,
                angularRateY, angularRateZ);

        final NEDFrame result21 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, kinematics);

        final NEDFrame result22 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, kinematics);

        final NEDFrame result23 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, kinematics);

        final NEDFrame result24 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, kinematics);

        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final NEDFrame result25 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result26 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result27 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result28 = navigator.navigateAndReturnNew(
                timeInterval, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result29 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final NEDFrame result30 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final NEDFrame result31 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final NEDFrame result32 = navigator.navigateAndReturnNew(
                timeInterval, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final Acceleration accelerationX = new Acceleration(fx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(fy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(fz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final NEDFrame result33 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result34 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result35 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result36 = navigator.navigateAndReturnNew(
                timeInterval, oldPosition, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result37 = navigator.navigateAndReturnNew(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result38 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result39 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result40 = navigator.navigateAndReturnNew(
                timeInterval, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX,
                angularRateY, angularRateZ);

        final NEDFrame result41 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result42 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result43 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVn, oldVe,
                oldVd, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result44 = navigator.navigateAndReturnNew(
                timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result45 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVelocity, fx, fy, fz, angularSpeedX,
                angularSpeedY, angularSpeedZ);

        final NEDFrame result46 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, fx, fy, fz, angularSpeedX, angularSpeedY,
                angularSpeedZ);

        final NEDFrame result47 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result48 = navigator.navigateAndReturnNew(
                timeInterval, oldPosition, oldC, oldVelocity,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result49 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final NEDFrame result50 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final NEDFrame result51 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result52 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result53 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY,
                accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result54 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY,
                accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result55 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC, oldSpeedN,
                oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result56 = navigator.navigateAndReturnNew(
                timeInterval, oldPosition, oldC, oldSpeedN,
                oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result57 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, accelerationX, accelerationY,
                accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result58 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, accelerationX, accelerationY,
                accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result59 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX,
                angularSpeedY, angularSpeedZ);

        final NEDFrame result60 = navigator.navigateAndReturnNew(
                timeInterval, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX,
                angularSpeedY, angularSpeedZ);

        final NEDFrame result61 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result62 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result63 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result64 = navigator.navigateAndReturnNew(
                timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularSpeedX,
                angularSpeedY, angularSpeedZ);

        final NEDFrame result65 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result66 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result67 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                kinematics);

        final NEDFrame result68 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                kinematics);

        final NEDFrame result69 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result70 = navigator.navigateAndReturnNew(
                timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result71 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, kinematics);

        final NEDFrame result72 = navigator.navigateAndReturnNew(
                timeInterval, oldFrame, kinematics);

        final NEDFrame result73 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result74 = navigator.navigateAndReturnNew(
                timeInterval, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result75 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result76 = navigator.navigateAndReturnNew(
                timeInterval, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result77 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result78 = navigator.navigateAndReturnNew(
                timeInterval, oldFrame,
                accelerationX, accelerationY, accelerationZ,
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
    }

    @Test
    public void testNavigateNED() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {

        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double oldLatitude = Math.toRadians(LATITUDE_DEGREES);
        final double oldLongitude = Math.toRadians(LONGITUDE_DEGREES);

        final double oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Quaternion q = new Quaternion(roll, pitch, yaw);

        final Matrix m = q.asInhomogeneousMatrix();
        final CoordinateTransformation oldC = new CoordinateTransformation(
                m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame oldFrame = new NEDFrame(oldLatitude, oldLongitude, HEIGHT,
                oldVn, oldVe, oldVd, oldC);

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

        final NEDFrame result1 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result1);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final NEDFrame result2 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result2);

        final NEDPosition oldPosition = new NEDPosition(oldLatitude, oldLongitude, HEIGHT);
        final NEDFrame result3 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVn, oldVe, oldVd,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result3);

        final NEDFrame result4 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result4);

        final NEDVelocity oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final NEDFrame result5 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result5);

        final NEDFrame result6 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result6);

        final NEDFrame result7 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result7);

        final NEDFrame result8 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result8);

        final NEDFrame result9 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, kinematics, result9);

        final NEDFrame result10 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, kinematics, result10);

        final NEDFrame result11 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVn, oldVe, oldVd, kinematics, result11);

        final NEDFrame result12 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC,
                oldVn, oldVe, oldVd, kinematics, result12);

        final NEDFrame result13 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVelocity, kinematics, result13);

        final NEDFrame result14 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVelocity, kinematics, result14);

        final NEDFrame result15 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                kinematics, result15);

        final NEDFrame result16 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldVelocity,
                kinematics, result16);

        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final Angle oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final Distance oldHeightDistance = new Distance(HEIGHT, DistanceUnit.METER);
        final NEDFrame result17 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result17);

        final NEDFrame result18 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result18);

        final NEDFrame result19 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, fx, fy, fz, angularRateX,
                angularRateY, angularRateZ, result19);

        final NEDFrame result20 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, fx, fy, fz, angularRateX,
                angularRateY, angularRateZ, result20);

        final NEDFrame result21 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, kinematics, result21);

        final NEDFrame result22 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, kinematics,
                result22);

        final NEDFrame result23 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, kinematics, result23);

        final NEDFrame result24 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, kinematics, result24);

        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final NEDFrame result25 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result25);

        final NEDFrame result26 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result26);

        final NEDFrame result27 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result27);

        final NEDFrame result28 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result28);

        final NEDFrame result29 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics,
                result29);

        final NEDFrame result30 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics,
                result30);

        final NEDFrame result31 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, kinematics, result31);

        final NEDFrame result32 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, kinematics, result32);

        final Acceleration accelerationX = new Acceleration(fx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(fy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(fz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final NEDFrame result33 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result33);

        final NEDFrame result34 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result34);

        final NEDFrame result35 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result35);

        final NEDFrame result36 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result36);

        final NEDFrame result37 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result37);

        final NEDFrame result38 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result38);

        final NEDFrame result39 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result39);

        final NEDFrame result40 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX,
                angularRateY, angularRateZ, result40);

        final NEDFrame result41 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result41);

        final NEDFrame result42 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result42);

        final NEDFrame result43 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVn, oldVe,
                oldVd, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ,
                result43);

        final NEDFrame result44 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ,
                result44);

        final NEDFrame result45 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVelocity, fx, fy, fz, angularSpeedX,
                angularSpeedY, angularSpeedZ, result45);

        final NEDFrame result46 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, fx, fy, fz, angularSpeedX, angularSpeedY,
                angularSpeedZ, result46);

        final NEDFrame result47 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ,
                result47);

        final NEDFrame result48 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldVelocity,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ,
                result48);

        final NEDFrame result49 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result49);

        final NEDFrame result50 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result50);

        final NEDFrame result51 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result51);

        final NEDFrame result52 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result52);

        final NEDFrame result53 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY,
                accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                result53);

        final NEDFrame result54 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY,
                accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ,
                result54);

        final NEDFrame result55 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldSpeedN,
                oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result55);

        final NEDFrame result56 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldSpeedN,
                oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result56);

        final NEDFrame result57 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, accelerationX, accelerationY,
                accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result57);

        final NEDFrame result58 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, accelerationX, accelerationY,
                accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ, result58);

        final NEDFrame result59 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX,
                angularSpeedY, angularSpeedZ, result59);

        final NEDFrame result60 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX,
                angularSpeedY, angularSpeedZ, result60);

        final NEDFrame result61 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result61);

        final NEDFrame result62 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result62);

        final NEDFrame result63 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result63);

        final NEDFrame result64 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularSpeedX,
                angularSpeedY, angularSpeedZ, result64);

        final NEDFrame result65 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result65);

        final NEDFrame result66 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result66);

        final NEDFrame result67 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                kinematics, result67);

        final NEDFrame result68 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                kinematics, result68);

        final NEDFrame result69 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result69);

        final NEDFrame result70 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result70);

        final NEDFrame result71 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldFrame, kinematics,
                result71);

        final NEDFrame result72 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldFrame, kinematics, result72);

        final NEDFrame result73 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result73);

        final NEDFrame result74 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result74);

        final NEDFrame result75 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result75);

        final NEDFrame result76 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result76);

        final NEDFrame result77 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result77);

        final NEDFrame result78 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result78);

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
    }

    @Test
    public void testNavigateNEDAndReturnNew() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {


        final UniformRandomizer randomizer = new UniformRandomizer(new Random());

        final double oldLatitude = Math.toRadians(LATITUDE_DEGREES);
        final double oldLongitude = Math.toRadians(LONGITUDE_DEGREES);

        final double oldVn = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double oldVe = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);
        final double oldVd = randomizer.nextDouble(MIN_VELOCITY_VALUE, MAX_VELOCITY_VALUE);

        final double roll = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(
                randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final Quaternion q = new Quaternion(roll, pitch, yaw);

        final Matrix m = q.asInhomogeneousMatrix();
        final CoordinateTransformation oldC = new CoordinateTransformation(
                m, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);

        final NEDFrame oldFrame = new NEDFrame(oldLatitude, oldLongitude, HEIGHT,
                oldVn, oldVe, oldVd, oldC);

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

        final NEDFrame result1 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final NEDFrame result2 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ);

        final NEDPosition oldPosition = new NEDPosition(oldLatitude, oldLongitude, HEIGHT);
        final NEDFrame result3 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVn, oldVe, oldVd,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final NEDFrame result4 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final NEDVelocity oldVelocity = new NEDVelocity(oldVn, oldVe, oldVd);
        final NEDFrame result5 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ);

        final NEDFrame result6 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ);

        final NEDFrame result7 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final NEDFrame result8 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldPosition, oldC, oldVelocity,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final NEDFrame result9 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, kinematics);

        final NEDFrame result10 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, kinematics);

        final NEDFrame result11 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVn, oldVe, oldVd, kinematics);

        final NEDFrame result12 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldPosition, oldC,
                oldVn, oldVe, oldVd, kinematics);

        final NEDFrame result13 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVelocity, kinematics);

        final NEDFrame result14 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVelocity, kinematics);

        final NEDFrame result15 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                kinematics);

        final NEDFrame result16 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldPosition, oldC, oldVelocity,
                kinematics);

        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final Angle oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final Distance oldHeightDistance = new Distance(HEIGHT, DistanceUnit.METER);
        final NEDFrame result17 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result18 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result19 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, fx, fy, fz, angularRateX,
                angularRateY, angularRateZ);

        final NEDFrame result20 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, fx, fy, fz, angularRateX,
                angularRateY, angularRateZ);

        final NEDFrame result21 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, kinematics);

        final NEDFrame result22 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, kinematics);

        final NEDFrame result23 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, kinematics);

        final NEDFrame result24 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, kinematics);

        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final NEDFrame result25 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result26 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result27 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result28 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result29 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final NEDFrame result30 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final NEDFrame result31 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final NEDFrame result32 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldPosition, oldC,
                oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final Acceleration accelerationX = new Acceleration(fx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(fy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(fz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final NEDFrame result33 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result34 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result35 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result36 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldPosition, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result37 = NEDInertialNavigator.navigateNEDAndReturnNew(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result38 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result39 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVelocity, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result40 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularRateX,
                angularRateY, angularRateZ);

        final NEDFrame result41 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result42 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result43 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVn, oldVe,
                oldVd, fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result44 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result45 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVelocity, fx, fy, fz, angularSpeedX,
                angularSpeedY, angularSpeedZ);

        final NEDFrame result46 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVelocity, fx, fy, fz, angularSpeedX, angularSpeedY,
                angularSpeedZ);

        final NEDFrame result47 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result48 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldPosition, oldC, oldVelocity,
                fx, fy, fz, angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result49 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final NEDFrame result50 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final NEDFrame result51 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result52 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result53 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY,
                accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result54 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, accelerationX, accelerationY,
                accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result55 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC, oldSpeedN,
                oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result56 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldPosition, oldC, oldSpeedN,
                oldSpeedE, oldSpeedD, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result57 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, accelerationX, accelerationY,
                accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result58 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVelocity, accelerationX, accelerationY,
                accelerationZ, angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result59 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX,
                angularSpeedY, angularSpeedZ);

        final NEDFrame result60 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldPosition, oldC, oldVelocity,
                accelerationX, accelerationY, accelerationZ, angularSpeedX,
                angularSpeedY, angularSpeedZ);

        final NEDFrame result61 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result62 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result63 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result64 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldPosition, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ, angularSpeedX,
                angularSpeedY, angularSpeedZ);

        final NEDFrame result65 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result66 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT, oldC,
                oldVn, oldVe, oldVd, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result67 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                kinematics);

        final NEDFrame result68 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                kinematics);

        final NEDFrame result69 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result70 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result71 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, kinematics);

        final NEDFrame result72 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldFrame, kinematics);

        final NEDFrame result73 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result74 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result75 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result76 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result77 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result78 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldFrame,
                accelerationX, accelerationY, accelerationZ,
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
    }

    @Test
    public void testNavigateFreeFallingBody()
            throws InvalidRotationMatrixException,
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

            final NEDFrame oldFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
            final NEDGravity gravity = NEDGravityEstimator
                    .estimateGravityAndReturnNew(latitude, height);
            final double fx = gravity.getGn();
            final double fy = gravity.getGe();
            final double fz = gravity.getGd();

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

            final NEDFrame newFrame = NEDInertialNavigator.navigateNEDAndReturnNew(
                    TIME_INTERVAL_SECONDS, oldFrame, kinematics);

            // Because body is in free fall its latitude and longitude does not change
            assertEquals(oldFrame.getLatitude(), newFrame.getLatitude(),
                    ABSOLUTE_ERROR);
            assertEquals(oldFrame.getLongitude(), newFrame.getLongitude(),
                    ABSOLUTE_ERROR);

            // Since body is falling, new height is smaller than the initial one
            assertTrue(newFrame.getHeight() < oldFrame.getHeight());

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testNavigateZeroTimeInterval()
            throws InvalidRotationMatrixException,
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

            final NEDFrame oldFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
            final NEDGravity gravity = NEDGravityEstimator
                    .estimateGravityAndReturnNew(latitude, height);
            final double fx = gravity.getGn();
            final double fy = gravity.getGe();
            final double fz = gravity.getGd();

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

            final NEDFrame newFrame = NEDInertialNavigator.navigateNEDAndReturnNew(
                    0.0, oldFrame, kinematics);

            // Because time interval is zero, body hasn't moved
            assertEquals(oldFrame.getLatitude(), newFrame.getLatitude(),
                    ABSOLUTE_ERROR);
            assertEquals(oldFrame.getLongitude(), newFrame.getLongitude(),
                    ABSOLUTE_ERROR);
            assertEquals(oldFrame.getHeight(), newFrame.getHeight(),
                    ABSOLUTE_ERROR);
            assertTrue(oldFrame.equals(newFrame, LARGE_ABSOLUTE_ERROR));

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testNavigateWithInitialVelocityAndNoSpecificForce()
            throws InvalidRotationMatrixException,
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

            final NEDFrame oldFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);

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

            final NEDFrame newFrame = NEDInertialNavigator.navigateNEDAndReturnNew(
                    TIME_INTERVAL_SECONDS, oldFrame, kinematics);

            // Because no specific force is applied to body, it will keep its inertia
            // (initial speed). This is approximate, since as the body moves the amount
            // of gravity applied to it changes as well
            final double oldSpeed = oldFrame.getVelocityNorm();
            double newSpeed = newFrame.getVelocityNorm();

            final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(oldFrame);
            final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter
                    .convertNEDtoECEFAndReturnNew(newFrame);

            final Point3D oldPosition = oldEcefFrame.getPosition();
            final Point3D newPosition = newEcefFrame.getPosition();

            final double distance = oldPosition.distanceTo(newPosition);

            final double estimatedSpeed = distance / TIME_INTERVAL_SECONDS;

            assertEquals(estimatedSpeed, newSpeed, VERY_LARGE_ABSOLUTE_ERROR);
            assertEquals(oldSpeed, newSpeed, 2.0 * VERY_LARGE_ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testNavigateWithNoInitialVelocityAndNoSpecificForce()
            throws InvalidRotationMatrixException,
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

            final NEDFrame oldFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);

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

            final NEDFrame newFrame = NEDInertialNavigator.navigateNEDAndReturnNew(
                    TIME_INTERVAL_SECONDS, oldFrame, kinematics);

            // Because no forces are applied to the body and the body has no initial velocity,
            // the body will remain on the same position
            assertEquals(oldFrame.getLatitude(), newFrame.getLatitude(),
                    ABSOLUTE_ERROR);
            assertEquals(oldFrame.getLongitude(), newFrame.getLongitude(),
                    ABSOLUTE_ERROR);
            assertEquals(oldFrame.getHeight(), newFrame.getHeight(),
                    VERY_LARGE_ABSOLUTE_ERROR);

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }

    @Test
    public void testCompareNavigations() throws InvalidRotationMatrixException,
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

            final NEDFrame oldFrame = new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
            final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(oldFrame);

            final NEDGravity gravity = NEDGravityEstimator
                    .estimateGravityAndReturnNew(latitude, height);
            final double fx = gravity.getGn();
            final double fy = gravity.getGe();
            final double fz = gravity.getGd();

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

            final NEDFrame newFrame1 = NEDInertialNavigator.navigateNEDAndReturnNew(
                    TIME_INTERVAL_SECONDS, oldFrame, kinematics);

            final ECEFFrame newEcefFrame = ECEFInertialNavigator
                    .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldEcefFrame,
                            kinematics);
            final NEDFrame newFrame2 = ECEFtoNEDFrameConverter
                    .convertECEFtoNEDAndReturnNew(newEcefFrame);

            // compare
            assertTrue(newFrame1.equals(newFrame2, LARGE_ABSOLUTE_ERROR));

            numValid++;
        }

        assertEquals(numValid, TIMES);
    }
}
