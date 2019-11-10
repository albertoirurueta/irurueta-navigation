package com.irurueta.navigation.inertial;

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

        final NEDFrame result3 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, kinematics, result3);

        final NEDFrame result4 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, kinematics, result4);

        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final Angle oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final Distance oldHeightDistance = new Distance(HEIGHT, DistanceUnit.METER);
        final NEDFrame result5 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result5);

        final NEDFrame result6 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result6);

        final NEDFrame result7 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, kinematics, result7);

        final NEDFrame result8 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, kinematics,
                result8);

        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final NEDFrame result9 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result9);

        final NEDFrame result10 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result10);

        final NEDFrame result11 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics,
                result11);

        final NEDFrame result12 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics,
                result12);

        final Acceleration accelerationX = new Acceleration(fx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(fy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(fz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final NEDFrame result13 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result13);

        final NEDFrame result14 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result14);

        final NEDFrame result15 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result15);

        final NEDFrame result16 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result16);

        final NEDFrame result17 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result17);

        final NEDFrame result18 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result18);

        final NEDFrame result19 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result19);

        final NEDFrame result20 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result20);

        final NEDFrame result21 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result21);

        final NEDFrame result22 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result22);

        final NEDFrame result23 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                kinematics, result23);

        final NEDFrame result24 = new NEDFrame();
        navigator.navigate(timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                kinematics, result24);

        final NEDFrame result25 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result25);

        final NEDFrame result26 = new NEDFrame();
        navigator.navigate(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result26);

        final NEDFrame result27 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, kinematics,
                result27);

        final NEDFrame result28 = new NEDFrame();
        navigator.navigate(timeInterval, oldFrame, kinematics, result28);

        final NEDFrame result29 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result29);

        final NEDFrame result30 = new NEDFrame();
        navigator.navigate(timeInterval, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result30);

        final NEDFrame result31 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result31);

        final NEDFrame result32 = new NEDFrame();
        navigator.navigate(timeInterval, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result32);

        final NEDFrame result33 = new NEDFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result33);

        final NEDFrame result34 = new NEDFrame();
        navigator.navigate(timeInterval, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result34);

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

        final NEDFrame result3 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, kinematics);

        final NEDFrame result4 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, kinematics);

        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final Angle oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final Distance oldHeightDistance = new Distance(HEIGHT, DistanceUnit.METER);
        final NEDFrame result5 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result6 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result7 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, kinematics);

        final NEDFrame result8 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, kinematics);

        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final NEDFrame result9 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result10 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result11 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final NEDFrame result12 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final Acceleration accelerationX = new Acceleration(fx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(fy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(fz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final NEDFrame result13 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result14 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result15 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result16 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result17 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final NEDFrame result18 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final NEDFrame result19 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result20 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result21 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result22 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result23 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                kinematics);

        final NEDFrame result24 = navigator.navigateAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                kinematics);

        final NEDFrame result25 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result26 = navigator.navigateAndReturnNew(
                timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result27 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, kinematics);

        final NEDFrame result28 = navigator.navigateAndReturnNew(
                timeInterval, oldFrame, kinematics);

        final NEDFrame result29 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result30 = navigator.navigateAndReturnNew(
                timeInterval, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result31 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result32 = navigator.navigateAndReturnNew(
                timeInterval, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result33 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result34 = navigator.navigateAndReturnNew(
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
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS,
                oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result1);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final NEDFrame result2 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval,
                oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result2);

        final NEDFrame result3 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS,
                oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, kinematics, result3);

        final NEDFrame result4 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval,
                oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, kinematics, result4);

        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final Angle oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final Distance oldHeightDistance = new Distance(HEIGHT, DistanceUnit.METER);
        final NEDFrame result5 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result5);

        final NEDFrame result6 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result6);

        final NEDFrame result7 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldC, oldVn, oldVe, oldVd, kinematics, result7);

        final NEDFrame result8 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldC, oldVn, oldVe, oldVd, kinematics, result8);

        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final NEDFrame result9 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS,
                oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result9);

        final NEDFrame result10 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval,
                oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result10);

        final NEDFrame result11 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS,
                oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics,
                result11);

        final NEDFrame result12 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval,
                oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics,
                result12);

        final Acceleration accelerationX = new Acceleration(fx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(fy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(fz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final NEDFrame result13 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS,
                oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result13);

        final NEDFrame result14 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval,
                oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result14);

        final NEDFrame result15 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS,
                oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result15);

        final NEDFrame result16 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval,
                oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result16);

        final NEDFrame result17 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result17);

        final NEDFrame result18 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result18);

        final NEDFrame result19 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result19);

        final NEDFrame result20 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result20);

        final NEDFrame result21 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result21);

        final NEDFrame result22 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result22);

        final NEDFrame result23 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                kinematics, result23);

        final NEDFrame result24 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval,
                oldLatitudeAngle, oldLongitudeAngle, oldHeightDistance,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                kinematics, result24);

        final NEDFrame result25 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS,
                oldFrame, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result25);

        final NEDFrame result26 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result26);

        final NEDFrame result27 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldFrame, kinematics,
                result27);

        final NEDFrame result28 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldFrame, kinematics, result28);

        final NEDFrame result29 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result29);

        final NEDFrame result30 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result30);

        final NEDFrame result31 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result31);

        final NEDFrame result32 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result32);

        final NEDFrame result33 = new NEDFrame();
        NEDInertialNavigator.navigateNED(TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result33);

        final NEDFrame result34 = new NEDFrame();
        NEDInertialNavigator.navigateNED(timeInterval, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result34);

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

        final NEDFrame result3 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, kinematics);

        final NEDFrame result4 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, kinematics);

        final Angle oldLatitudeAngle = new Angle(oldLatitude, AngleUnit.RADIANS);
        final Angle oldLongitudeAngle = new Angle(oldLongitude, AngleUnit.RADIANS);
        final Distance oldHeightDistance = new Distance(HEIGHT, DistanceUnit.METER);
        final NEDFrame result5 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result6 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result7 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, kinematics);

        final NEDFrame result8 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd, kinematics);

        final Speed oldSpeedN = new Speed(oldVn, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedE = new Speed(oldVe, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedD = new Speed(oldVd, SpeedUnit.METERS_PER_SECOND);
        final NEDFrame result9 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result10 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldSpeedN, oldSpeedE, oldSpeedD, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result11 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final NEDFrame result12 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldSpeedN, oldSpeedE, oldSpeedD, kinematics);

        final Acceleration accelerationX = new Acceleration(fx,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(fy,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(fz,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final NEDFrame result13 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result14 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitude, oldLongitude,
                HEIGHT, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result15 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result16 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitude, oldLongitude, HEIGHT,
                oldC, oldVn, oldVe, oldVd, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result17 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final NEDFrame result18 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final NEDFrame result19 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result20 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result21 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result22 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldVn, oldVe, oldVd,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result23 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                kinematics);

        final NEDFrame result24 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldLatitudeAngle, oldLongitudeAngle,
                oldHeightDistance, oldC, oldSpeedN, oldSpeedE, oldSpeedD,
                kinematics);

        final NEDFrame result25 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result26 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result27 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, kinematics);

        final NEDFrame result28 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldFrame, kinematics);

        final NEDFrame result29 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result30 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final NEDFrame result31 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result32 = NEDInertialNavigator.navigateNEDAndReturnNew(
                timeInterval, oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result33 = NEDInertialNavigator.navigateNEDAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final NEDFrame result34 = NEDInertialNavigator.navigateNEDAndReturnNew(
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
