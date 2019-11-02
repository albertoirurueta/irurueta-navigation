package com.irurueta.navigation.inertial;

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
import com.irurueta.navigation.inertial.estimators.GravityECEFEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.*;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class InertialECEFNavigatorTest {
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
        InertialECEFNavigator.navigateECEF(0.0,
                0.0, 0.0, 0.0, c,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0,
                0.0, 0.0, 0.0, result);
    }

    @Test
    public void testNavigate() throws InvalidRotationMatrixException,
            InvalidSourceAndDestinationFrameTypeException, InertialNavigatorException {
        final InertialECEFNavigator navigator = new InertialECEFNavigator();

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

        final Kinematics kinematics = new Kinematics(fx, fy, fz,
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

        final ECEFFrame result3 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, kinematics, result3);

        final ECEFFrame result4 = new ECEFFrame();
        navigator.navigate(timeInterval, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, kinematics, result4);

        final Point3D oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);
        final ECEFFrame result5 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result5);

        final ECEFFrame result6 = new ECEFFrame();
        navigator.navigate(timeInterval, oldPosition,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result6);

        final ECEFFrame result7 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS, oldPosition,
                oldC, oldVx, oldVy, oldVz, kinematics, result7);

        final ECEFFrame result8 = new ECEFFrame();
        navigator.navigate(timeInterval, oldPosition,
                oldC, oldVx, oldVy, oldVz, kinematics, result8);

        final Distance oldDistanceX = new Distance(oldX, DistanceUnit.METER);
        final Distance oldDistanceY = new Distance(oldY, DistanceUnit.METER);
        final Distance oldDistanceZ = new Distance(oldZ, DistanceUnit.METER);
        final ECEFFrame result9 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result9);

        final ECEFFrame result10 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result10);

        final ECEFFrame result11 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics, result11);

        final ECEFFrame result12 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics, result12);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);
        final ECEFFrame result13 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result13);

        final ECEFFrame result14 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result14);

        final ECEFFrame result15 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result15);

        final ECEFFrame result16 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result16);

        final Acceleration accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final ECEFFrame result17 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result17);

        final ECEFFrame result18 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result18);

        final ECEFFrame result19 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result19);

        final ECEFFrame result20 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result20);

        final ECEFFrame result21 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result21);

        final ECEFFrame result22 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result22);

        final ECEFFrame result23 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result23);

        final ECEFFrame result24 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result24);

        final ECEFFrame result25 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result25);

        final ECEFFrame result26 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result26);

        final ECEFFrame result27 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics,
                result27);

        final ECEFFrame result28 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics,
                result28);

        final ECEFFrame result29 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result29);

        final ECEFFrame result30 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result30);

        final ECEFFrame result31 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldFrame, kinematics, result31);

        final ECEFFrame result32 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldFrame, kinematics, result32);

        final ECEFFrame result33 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result33);

        final ECEFFrame result34 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result34);

        final ECEFFrame result35 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result35);

        final ECEFFrame result36 = new ECEFFrame();
        navigator.navigate(timeInterval,
                oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result36);

        final ECEFFrame result37 = new ECEFFrame();
        navigator.navigate(TIME_INTERVAL_SECONDS,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result37);

        final ECEFFrame result38 = new ECEFFrame();
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
        final InertialECEFNavigator navigator = new InertialECEFNavigator();

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

        final Kinematics kinematics = new Kinematics(fx, fy, fz,
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

        final ECEFFrame result3 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, kinematics);

        final ECEFFrame result4 = navigator.navigateAndReturnNew(
                timeInterval, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, kinematics);

        final Point3D oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);
        final ECEFFrame result5 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result6 = navigator.navigateAndReturnNew(
                timeInterval, oldPosition,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result7 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldPosition,
                oldC, oldVx, oldVy, oldVz, kinematics);

        final ECEFFrame result8 = navigator.navigateAndReturnNew(
                timeInterval, oldPosition,
                oldC, oldVx, oldVy, oldVz, kinematics);

        final Distance oldDistanceX = new Distance(oldX, DistanceUnit.METER);
        final Distance oldDistanceY = new Distance(oldY, DistanceUnit.METER);
        final Distance oldDistanceZ = new Distance(oldZ, DistanceUnit.METER);
        final ECEFFrame result9 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result10 = navigator.navigateAndReturnNew(
                timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result11 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics);

        final ECEFFrame result12 = navigator.navigateAndReturnNew(
                timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);
        final ECEFFrame result13 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result14 = navigator.navigateAndReturnNew(
                timeInterval,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result15 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics);

        final ECEFFrame result16 = navigator.navigateAndReturnNew(
                timeInterval,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics);

        final Acceleration accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final ECEFFrame result17 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result18 = navigator.navigateAndReturnNew(
                timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result19 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result20 = navigator.navigateAndReturnNew(
                timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result21 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result22 = navigator.navigateAndReturnNew(
                timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result23 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result24 = navigator.navigateAndReturnNew(
                timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result25 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result26 = navigator.navigateAndReturnNew(
                timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result27 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final ECEFFrame result28 = navigator.navigateAndReturnNew(
                timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final ECEFFrame result29 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result30 = navigator.navigateAndReturnNew(
                timeInterval,
                oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result31 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS, oldFrame, kinematics);

        final ECEFFrame result32 = navigator.navigateAndReturnNew(
                timeInterval, oldFrame, kinematics);

        final ECEFFrame result33 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result34 = navigator.navigateAndReturnNew(
                timeInterval,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result35 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result36 = navigator.navigateAndReturnNew(
                timeInterval,
                oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result37 = navigator.navigateAndReturnNew(
                TIME_INTERVAL_SECONDS,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result38 = navigator.navigateAndReturnNew(
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

        final Kinematics kinematics = new Kinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result1 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result1);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final ECEFFrame result2 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result2);

        final ECEFFrame result3 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, kinematics, result3);

        final ECEFFrame result4 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(timeInterval, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, kinematics, result4);

        final Point3D oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);
        final ECEFFrame result5 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldPosition,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result5);

        final ECEFFrame result6 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(timeInterval, oldPosition,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result6);

        final ECEFFrame result7 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(TIME_INTERVAL_SECONDS, oldPosition,
                oldC, oldVx, oldVy, oldVz, kinematics, result7);

        final ECEFFrame result8 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(timeInterval, oldPosition,
                oldC, oldVx, oldVy, oldVz, kinematics, result8);

        final Distance oldDistanceX = new Distance(oldX, DistanceUnit.METER);
        final Distance oldDistanceY = new Distance(oldY, DistanceUnit.METER);
        final Distance oldDistanceZ = new Distance(oldZ, DistanceUnit.METER);
        final ECEFFrame result9 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result9);

        final ECEFFrame result10 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result10);

        final ECEFFrame result11 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics, result11);

        final ECEFFrame result12 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldVx, oldVy, oldVz, kinematics, result12);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);
        final ECEFFrame result13 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result13);

        final ECEFFrame result14 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(timeInterval,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result14);

        final ECEFFrame result15 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result15);

        final ECEFFrame result16 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(timeInterval,
                oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result16);

        final Acceleration accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final ECEFFrame result17 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result17);

        final ECEFFrame result18 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result18);

        final ECEFFrame result19 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result19);

        final ECEFFrame result20 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result20);

        final ECEFFrame result21 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result21);

        final ECEFFrame result22 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result22);

        final ECEFFrame result23 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result23);

        final ECEFFrame result24 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result24);

        final ECEFFrame result25 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result25);

        final ECEFFrame result26 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(timeInterval,
                oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result26);

        final ECEFFrame result27 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics,
                result27);

        final ECEFFrame result28 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(timeInterval,
                oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics,
                result28);

        final ECEFFrame result29 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result29);

        final ECEFFrame result30 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(timeInterval,
                oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result30);

        final ECEFFrame result31 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldFrame, kinematics, result31);

        final ECEFFrame result32 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(timeInterval,
                oldFrame, kinematics, result32);

        final ECEFFrame result33 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result33);

        final ECEFFrame result34 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(timeInterval,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularRateX, angularRateY, angularRateZ, result34);

        final ECEFFrame result35 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result35);

        final ECEFFrame result36 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(timeInterval,
                oldFrame, fx, fy, fz,
                angularSpeedX, angularSpeedY, angularSpeedZ, result36);

        final ECEFFrame result37 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(TIME_INTERVAL_SECONDS,
                oldFrame, accelerationX, accelerationY, accelerationZ,
                angularSpeedX, angularSpeedY, angularSpeedZ, result37);

        final ECEFFrame result38 = new ECEFFrame();
        InertialECEFNavigator.navigateECEF(timeInterval,
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

        final Kinematics kinematics = new Kinematics(fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result1 = InertialECEFNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                        oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final ECEFFrame result2 = InertialECEFNavigator
                .navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ,
                        oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result3 = InertialECEFNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldX, oldY, oldZ,
                        oldC, oldVx, oldVy, oldVz, kinematics);

        final ECEFFrame result4 = InertialECEFNavigator
                .navigateECEFAndReturnNew(timeInterval, oldX, oldY, oldZ,
                        oldC, oldVx, oldVy, oldVz, kinematics);

        final Point3D oldPosition = new InhomogeneousPoint3D(oldX, oldY, oldZ);
        final ECEFFrame result5 = InertialECEFNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition,
                        oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result6 = InertialECEFNavigator
                .navigateECEFAndReturnNew(timeInterval, oldPosition,
                        oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result7 = InertialECEFNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS, oldPosition,
                        oldC, oldVx, oldVy, oldVz, kinematics);

        final ECEFFrame result8 = InertialECEFNavigator
                .navigateECEFAndReturnNew(timeInterval, oldPosition,
                        oldC, oldVx, oldVy, oldVz, kinematics);

        final Distance oldDistanceX = new Distance(oldX, DistanceUnit.METER);
        final Distance oldDistanceY = new Distance(oldY, DistanceUnit.METER);
        final Distance oldDistanceZ = new Distance(oldZ, DistanceUnit.METER);
        final ECEFFrame result9 = InertialECEFNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldVx, oldVy, oldVz, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result10 = InertialECEFNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldVx, oldVy, oldVz, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result11 = InertialECEFNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldVx, oldVy, oldVz, kinematics);

        final ECEFFrame result12 = InertialECEFNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldVx, oldVy, oldVz, kinematics);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);
        final ECEFFrame result13 = InertialECEFNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                        fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result14 = InertialECEFNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                        fx, fy, fz, angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result15 = InertialECEFNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                        kinematics);

        final ECEFFrame result16 = InertialECEFNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                        kinematics);

        final Acceleration accelerationX = new Acceleration(fx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationY = new Acceleration(fy, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final Acceleration accelerationZ = new Acceleration(fz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final ECEFFrame result17 = InertialECEFNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                        accelerationX, accelerationY, accelerationZ,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result18 = InertialECEFNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                        accelerationX, accelerationY, accelerationZ,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result19 = InertialECEFNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result20 = InertialECEFNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result21 = InertialECEFNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result22 = InertialECEFNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result23 = InertialECEFNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ,
                        accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result24 = InertialECEFNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ,
                        accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result25 = InertialECEFNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                        accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result26 = InertialECEFNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                        accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result27 = InertialECEFNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final ECEFFrame result28 = InertialECEFNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldDistanceX, oldDistanceY, oldDistanceZ, oldC,
                        oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);

        final ECEFFrame result29 = InertialECEFNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldFrame, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result30 = InertialECEFNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldFrame, fx, fy, fz,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result31 = InertialECEFNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldFrame, kinematics);

        final ECEFFrame result32 = InertialECEFNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldFrame, kinematics);

        final ECEFFrame result33 = InertialECEFNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldFrame, accelerationX, accelerationY, accelerationZ,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result34 = InertialECEFNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldFrame, accelerationX, accelerationY, accelerationZ,
                        angularRateX, angularRateY, angularRateZ);

        final ECEFFrame result35 = InertialECEFNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldFrame, fx, fy, fz,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result36 = InertialECEFNavigator
                .navigateECEFAndReturnNew(timeInterval,
                        oldFrame, fx, fy, fz,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result37 = InertialECEFNavigator
                .navigateECEFAndReturnNew(TIME_INTERVAL_SECONDS,
                        oldFrame, accelerationX, accelerationY, accelerationZ,
                        angularSpeedX, angularSpeedY, angularSpeedZ);

        final ECEFFrame result38 = InertialECEFNavigator
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
            final GravityECEF gravity = GravityECEFEstimator.estimateGravityAndReturnNew(oldFrame);
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

            final Kinematics kinematics = new Kinematics(fx, fy, fz,
                    angularRateX, angularRateY, angularRateZ);

            final ECEFFrame newFrame = InertialECEFNavigator.navigateECEFAndReturnNew(
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
            final GravityECEF gravity = GravityECEFEstimator.estimateGravityAndReturnNew(oldFrame);
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

            final Kinematics kinematics = new Kinematics(fx, fy, fz,
                    angularRateX, angularRateY, angularRateZ);

            final ECEFFrame newFrame = InertialECEFNavigator.navigateECEFAndReturnNew(
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

            final Kinematics kinematics = new Kinematics(fx, fy, fz,
                    angularRateX, angularRateY, angularRateZ);

            final ECEFFrame newFrame = InertialECEFNavigator.navigateECEFAndReturnNew(
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

            final Kinematics kinematics = new Kinematics(fx, fy, fz,
                    angularRateX, angularRateY, angularRateZ);

            final ECEFFrame newFrame = InertialECEFNavigator.navigateECEFAndReturnNew(
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
