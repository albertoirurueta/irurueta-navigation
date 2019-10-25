package com.irurueta.navigation.inertial.estimators;

import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Point3D;
import com.irurueta.geometry.Quaternion;
import com.irurueta.navigation.frames.CoordinateTransformationMatrix;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.Kinematics;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.Random;

import static org.junit.Assert.assertEquals;

public class KinematicsEstimatorTest {

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

    @Test
    public void testEstimate()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(oldNedFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(newNedFrame);


        final CoordinateTransformationMatrix c = newEcefFrame
                .getCoordinateTransformationMatrix();
        final CoordinateTransformationMatrix oldC = oldEcefFrame
                .getCoordinateTransformationMatrix();

        final double vx = newEcefFrame.getVx();
        final double vy = newEcefFrame.getVy();
        final double vz = newEcefFrame.getVz();

        final double oldVx = oldEcefFrame.getVx();
        final double oldVy = oldEcefFrame.getVy();
        final double oldVz = oldEcefFrame.getVz();

        final double x = newEcefFrame.getX();
        final double y = newEcefFrame.getY();
        final double z = newEcefFrame.getZ();

        final KinematicsEstimator estimator = new KinematicsEstimator();

        final Kinematics k1 = new Kinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, k1);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final Kinematics k2 = new Kinematics();
        estimator.estimate(timeInterval, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, k2);

        final Kinematics k3 = new Kinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, newEcefFrame,
                oldC, oldVx, oldVy, oldVz, k3);

        final Kinematics k4 = new Kinematics();
        estimator.estimate(timeInterval, newEcefFrame,
                oldC, oldVx, oldVy, oldVz, k4);

        final Kinematics k5 = new Kinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, newEcefFrame,
                oldEcefFrame, k5);

        final Kinematics k6 = new Kinematics();
        estimator.estimate(timeInterval, newEcefFrame,
                oldEcefFrame, k6);

        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        final Kinematics k7 = new Kinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z, k7);

        final Kinematics k8 = new Kinematics();
        estimator.estimate(timeInterval, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z, k8);

        final Kinematics k9 = new Kinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, newEcefFrame,
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ, k9);

        final Kinematics k10 = new Kinematics();
        estimator.estimate(timeInterval, newEcefFrame,
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ, k10);

        final Point3D position = new InhomogeneousPoint3D(x, y, z);
        final Kinematics k11 = new Kinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, position, k11);

        final Kinematics k12 = new Kinematics();
        estimator.estimate(timeInterval, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, position, k12);

        final Kinematics k13 = new Kinematics();
        estimator.estimate(TIME_INTERVAL_SECONDS, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position, k13);

        final Kinematics k14 = new Kinematics();
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

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(newNedFrame);


        final CoordinateTransformationMatrix c = newEcefFrame
                .getCoordinateTransformationMatrix();
        final CoordinateTransformationMatrix oldC = oldEcefFrame
                .getCoordinateTransformationMatrix();

        final double vx = newEcefFrame.getVx();
        final double vy = newEcefFrame.getVy();
        final double vz = newEcefFrame.getVz();

        final double oldVx = oldEcefFrame.getVx();
        final double oldVy = oldEcefFrame.getVy();
        final double oldVz = oldEcefFrame.getVz();

        final double x = newEcefFrame.getX();
        final double y = newEcefFrame.getY();
        final double z = newEcefFrame.getZ();

        final KinematicsEstimator estimator = new KinematicsEstimator();

        final Kinematics k1 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final Kinematics k2 = estimator.estimateAndReturnNew(timeInterval, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        final Kinematics k3 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, newEcefFrame, oldC, oldVx, oldVy, oldVz);

        final Kinematics k4 = estimator.estimateAndReturnNew(timeInterval, newEcefFrame,
                oldC, oldVx, oldVy, oldVz);

        final Kinematics k5 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame);

        final Kinematics k6 = estimator.estimateAndReturnNew(timeInterval, newEcefFrame,
                oldEcefFrame);

        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        final Kinematics k7 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z);

        final Kinematics k8 = estimator.estimateAndReturnNew(timeInterval, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z);

        final Kinematics k9 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, newEcefFrame,
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ);

        final Kinematics k10 = estimator.estimateAndReturnNew(
                timeInterval, newEcefFrame,
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ);

        final Point3D position = new InhomogeneousPoint3D(x, y, z);
        final Kinematics k11 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, position);

        final Kinematics k12 = estimator.estimateAndReturnNew(timeInterval, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, position);

        final Kinematics k13 = estimator.estimateAndReturnNew(
                TIME_INTERVAL_SECONDS, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position);

        final Kinematics k14 = estimator.estimateAndReturnNew(timeInterval, c, oldC,
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
            InvalidRotationMatrixException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(oldNedFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(newNedFrame);


        final CoordinateTransformationMatrix c = newEcefFrame
                .getCoordinateTransformationMatrix();
        final CoordinateTransformationMatrix oldC = oldEcefFrame
                .getCoordinateTransformationMatrix();

        final double vx = newEcefFrame.getVx();
        final double vy = newEcefFrame.getVy();
        final double vz = newEcefFrame.getVz();

        final double oldVx = oldEcefFrame.getVx();
        final double oldVy = oldEcefFrame.getVy();
        final double oldVz = oldEcefFrame.getVz();

        final double x = newEcefFrame.getX();
        final double y = newEcefFrame.getY();
        final double z = newEcefFrame.getZ();

        final Kinematics k1 = new Kinematics();
        KinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, k1);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final Kinematics k2 = new Kinematics();
        KinematicsEstimator.estimateKinematics(timeInterval, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, k2);

        final Kinematics k3 = new Kinematics();
        KinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame,
                oldC, oldVx, oldVy, oldVz, k3);

        final Kinematics k4 = new Kinematics();
        KinematicsEstimator.estimateKinematics(timeInterval, newEcefFrame,
                oldC, oldVx, oldVy, oldVz, k4);

        final Kinematics k5 = new Kinematics();
        KinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame,
                oldEcefFrame, k5);

        final Kinematics k6 = new Kinematics();
        KinematicsEstimator.estimateKinematics(timeInterval, newEcefFrame,
                oldEcefFrame, k6);

        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        final Kinematics k7 = new Kinematics();
        KinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z, k7);

        final Kinematics k8 = new Kinematics();
        KinematicsEstimator.estimateKinematics(timeInterval, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z, k8);

        final Kinematics k9 = new Kinematics();
        KinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame,
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ, k9);

        final Kinematics k10 = new Kinematics();
        KinematicsEstimator.estimateKinematics(timeInterval, newEcefFrame,
                oldC, oldSpeedX, oldSpeedY, oldSpeedZ, k10);

        final Point3D position = new InhomogeneousPoint3D(x, y, z);
        final Kinematics k11 = new Kinematics();
        KinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, position, k11);

        final Kinematics k12 = new Kinematics();
        KinematicsEstimator.estimateKinematics(timeInterval, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, position, k12);

        final Kinematics k13 = new Kinematics();
        KinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, c, oldC,
                speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position, k13);

        final Kinematics k14 = new Kinematics();
        KinematicsEstimator.estimateKinematics(timeInterval, c, oldC,
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
    public void testEstimateKinematicsAndReturnNew()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(oldNedFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(newNedFrame);


        final CoordinateTransformationMatrix c = newEcefFrame
                .getCoordinateTransformationMatrix();
        final CoordinateTransformationMatrix oldC = oldEcefFrame
                .getCoordinateTransformationMatrix();

        final double vx = newEcefFrame.getVx();
        final double vy = newEcefFrame.getVy();
        final double vz = newEcefFrame.getVz();

        final double oldVx = oldEcefFrame.getVx();
        final double oldVy = oldEcefFrame.getVy();
        final double oldVz = oldEcefFrame.getVz();

        final double x = newEcefFrame.getX();
        final double y = newEcefFrame.getY();
        final double z = newEcefFrame.getZ();

        final Kinematics k1 = KinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                        vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        final Time timeInterval = new Time(TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        final Kinematics k2 = KinematicsEstimator
                .estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                        vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);

        final Kinematics k3 = KinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame,
                        oldC, oldVx, oldVy, oldVz);

        final Kinematics k4 = KinematicsEstimator
                .estimateKinematicsAndReturnNew(timeInterval, newEcefFrame,
                        oldC, oldVx, oldVy, oldVz);

        final Kinematics k5 = KinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame,
                        oldEcefFrame);

        final Kinematics k6 = KinematicsEstimator
                .estimateKinematicsAndReturnNew(timeInterval, newEcefFrame,
                        oldEcefFrame);

        final Speed speedX = new Speed(vx, SpeedUnit.METERS_PER_SECOND);
        final Speed speedY = new Speed(vy, SpeedUnit.METERS_PER_SECOND);
        final Speed speedZ = new Speed(vz, SpeedUnit.METERS_PER_SECOND);

        final Speed oldSpeedX = new Speed(oldVx, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedY = new Speed(oldVy, SpeedUnit.METERS_PER_SECOND);
        final Speed oldSpeedZ = new Speed(oldVz, SpeedUnit.METERS_PER_SECOND);

        final Kinematics k7 = KinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                        speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z);

        final Kinematics k8 = KinematicsEstimator
                .estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                        speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, x, y, z);

        final Kinematics k9 = KinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, newEcefFrame,
                        oldC, oldSpeedX, oldSpeedY, oldSpeedZ);

        final Kinematics k10 = KinematicsEstimator
                .estimateKinematicsAndReturnNew(timeInterval, newEcefFrame,
                        oldC, oldSpeedX, oldSpeedY, oldSpeedZ);

        final Point3D position = new InhomogeneousPoint3D(x, y, z);
        final Kinematics k11 = KinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                        vx, vy, vz, oldVx, oldVy, oldVz, position);

        final Kinematics k12 = KinematicsEstimator
                .estimateKinematicsAndReturnNew(timeInterval, c, oldC,
                        vx, vy, vz, oldVx, oldVy, oldVz, position);

        final Kinematics k13 = KinematicsEstimator
                .estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                        speedX, speedY, speedZ, oldSpeedX, oldSpeedY, oldSpeedZ, position);

        final Kinematics k14 = KinematicsEstimator
                .estimateKinematicsAndReturnNew(timeInterval, c, oldC,
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

    @Test(expected = IllegalArgumentException.class)
    public void testEstimateKinematicsWhenNegativeIntervalThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(oldNedFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(newNedFrame);


        final CoordinateTransformationMatrix c = newEcefFrame
                .getCoordinateTransformationMatrix();
        final CoordinateTransformationMatrix oldC = oldEcefFrame
                .getCoordinateTransformationMatrix();

        final double vx = newEcefFrame.getVx();
        final double vy = newEcefFrame.getVy();
        final double vz = newEcefFrame.getVz();

        final double oldVx = oldEcefFrame.getVx();
        final double oldVy = oldEcefFrame.getVy();
        final double oldVz = oldEcefFrame.getVz();

        final double x = newEcefFrame.getX();
        final double y = newEcefFrame.getY();
        final double z = newEcefFrame.getZ();

        KinematicsEstimator.estimateKinematicsAndReturnNew(-1.0, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);
    }

    @Test(expected = IllegalArgumentException.class)
    public void testEstimateKinematicsWhenInvalidCoordinateTransformationMatrixThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(oldNedFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(newNedFrame);


        final CoordinateTransformationMatrix c = newEcefFrame
                .getCoordinateTransformationMatrix();
        final CoordinateTransformationMatrix oldC = oldEcefFrame
                .getCoordinateTransformationMatrix();

        final double vx = newEcefFrame.getVx();
        final double vy = newEcefFrame.getVy();
        final double vz = newEcefFrame.getVz();

        final double oldVx = oldEcefFrame.getVx();
        final double oldVy = oldEcefFrame.getVy();
        final double oldVz = oldEcefFrame.getVz();

        final double x = newEcefFrame.getX();
        final double y = newEcefFrame.getY();
        final double z = newEcefFrame.getZ();

        c.setDestinationType(FrameType.BODY_FRAME);
        KinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);
    }

    @Test(expected = IllegalArgumentException.class)
    public void testEstimateKinematicsWhenInvalidOldCoordinateTransformationMatrixThrowsIllegalArgumentException()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(oldNedFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(newNedFrame);


        final CoordinateTransformationMatrix c = newEcefFrame
                .getCoordinateTransformationMatrix();
        final CoordinateTransformationMatrix oldC = oldEcefFrame
                .getCoordinateTransformationMatrix();

        final double vx = newEcefFrame.getVx();
        final double vy = newEcefFrame.getVy();
        final double vz = newEcefFrame.getVz();

        final double oldVx = oldEcefFrame.getVx();
        final double oldVy = oldEcefFrame.getVy();
        final double oldVz = oldEcefFrame.getVz();

        final double x = newEcefFrame.getX();
        final double y = newEcefFrame.getY();
        final double z = newEcefFrame.getZ();

        oldC.setDestinationType(FrameType.BODY_FRAME);
        KinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, c, oldC,
                vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);
    }

    @Test
    public void testEstimateKinematicsWhenZeroTimeIntervalReturnsZeroValues()
            throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException {

        final NEDFrame oldNedFrame = createOldNedFrame();
        final ECEFFrame oldEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(oldNedFrame);

        final NEDFrame newNedFrame = createNewNedFrame(oldNedFrame);
        final ECEFFrame newEcefFrame = NEDtoECEFFrameConverter
                .convertNEDtoECEFAndReturnNew(newNedFrame);


        final CoordinateTransformationMatrix c = newEcefFrame
                .getCoordinateTransformationMatrix();
        final CoordinateTransformationMatrix oldC = oldEcefFrame
                .getCoordinateTransformationMatrix();

        final double vx = newEcefFrame.getVx();
        final double vy = newEcefFrame.getVy();
        final double vz = newEcefFrame.getVz();

        final double oldVx = oldEcefFrame.getVx();
        final double oldVy = oldEcefFrame.getVy();
        final double oldVz = oldEcefFrame.getVz();

        final double x = newEcefFrame.getX();
        final double y = newEcefFrame.getY();
        final double z = newEcefFrame.getZ();

        final Kinematics k = KinematicsEstimator
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
        final CoordinateTransformationMatrix c = new CoordinateTransformationMatrix(
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

        final CoordinateTransformationMatrix oldC = oldFrame
                .getCoordinateTransformationMatrix();

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
        final CoordinateTransformationMatrix c = new CoordinateTransformationMatrix(
                m, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        return new NEDFrame(latitude, longitude, height, vn, ve, vd, c);
    }
}
