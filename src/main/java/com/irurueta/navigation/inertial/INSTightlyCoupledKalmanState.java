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
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.gnss.ECEFPositionAndVelocity;
import com.irurueta.navigation.gnss.GNSSEstimation;
import com.irurueta.units.*;

import java.io.Serializable;
import java.util.Objects;

/**
 * Kalman filter state for tightly coupled INS/GNSS extended Kalman filter.
 */
public class INSTightlyCoupledKalmanState implements Serializable, Cloneable {

    /**
     * Number of parameters of the Kalman filter.
     */
    public static final int NUM_PARAMS = 17;

    /**
     * Estimated body to ECEF coordinate transformation matrix.
     */
    private Matrix mBodyToEcefCoordinateTransformationMatrix;

    /**
     * Estimated ECEF user velocity resolved around x axis and expressed in meters per second (m/s).
     */
    private double mVx;

    /**
     * Estimated ECEF user velocity resolved around y axis and expressed in meters per second (m/s).
     */
    private double mVy;

    /**
     * Estimated ECEF user velocity resolved around z axis and expressed in meters per second (m/s).
     */
    private double mVz;

    /**
     * X coordinate of estimated ECEF user position expressed in meters (m).
     */
    private double mX;

    /**
     * Y coordinate of estimated ECEF user position expressed in meters (m).
     */
    private double mY;

    /**
     * Z coordinate of estimated ECEF user position expressed in meters (m).
     */
    private double mZ;

    /**
     * Estimated accelerometer bias resolved around x axis and expressed in
     * meters per squared second (m/s^2).
     */
    private double mAccelerationBiasX;

    /**
     * Estimated accelerometer bias resolved around y axis and expressed in
     * meters per squared second (m/s^2).
     */
    private double mAccelerationBiasY;

    /**
     * Estimated accelerometer bias resolved around z axis and expressed in
     * meters per squared second (m/s^2).
     */
    private double mAccelerationBiasZ;

    /**
     * Estimated gyroscope bias resolved around x axis and expressed in
     * radians per second (rad/s).
     */
    private double mGyroBiasX;

    /**
     * Estimated gyroscope bias resolved around y axis and expressed in
     * radians per second (rad/s).
     */
    private double mGyroBiasY;

    /**
     * Estimated gyroscope bias resolved around z axis and expressed in
     * radians per second (rad/s).
     */
    private double mGyroBiasZ;

    /**
     * Estimated receiver clock offset expressed in meters (m).
     */
    private double mReceiverClockOffset;

    /**
     * Estimated receiver clock drift expressed in meters per second (m/s).
     */
    private double mReceiverClockDrift;

    /**
     * Estimated Kalman filter error covariance matrix.
     */
    private Matrix mCovariance;

    /**
     * Constructor.
     */
    public INSTightlyCoupledKalmanState() {
    }

    /**
     * Constructor.
     *
     * @param bodyToEcefCoordinateTransformationMatrix estimated body to ECEF coordinate transformation matrix.
     * @param vx                                       estimated ECEF user velocity resolved around x axis and
     *                                                 expressed in meters per second (m/s).
     * @param vy                                       estimated ECEF user velocity resolved around y axis and
     *                                                 expressed in meters per second (m/s).
     * @param vz                                       estimated ECEF user velocity resolved around z axis and
     *                                                 expressed in meters per second (m/s).
     * @param x                                        x coordinate of estimated ECEF user position expressed
     *                                                 in meters (m).
     * @param y                                        y coordinate of estimated ECEF user position expressed
     *                                                 in meters (m).
     * @param z                                        z coordinate of estimated ECEF user position expressed
     *                                                 in meters (m).
     * @param accelerationBiasX                        estimated accelerometer bias resolved around x axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY                        estimated accelerometer bias resolved around y axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ                        estimated accelerometer bias resolved around z axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param gyroBiasX                                estimated gyroscope bias resolved around x axis and
     *                                                 expressed in radians per second (rad/s).
     * @param gyroBiasY                                estimated gyroscope bias resolved around y axis and
     *                                                 expressed in radians per second (rad/s).
     * @param gyroBiasZ                                estimated gyroscope bias resolved around z axis and
     *                                                 expressed in radians per second (rad/s).
     * @param receiverClockOffset                      estimated receiver clock offset expressed in meters (m).
     * @param receiverClockDrift                       estimated receiver clock drift expressed in meters per
     *                                                 second (m/s).
     * @param covariance                               estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided body to ECEF coordinate transformation matrix is not 3x3
     *                                  or if provided covariance matrix is not 17x17.
     */
    public INSTightlyCoupledKalmanState(
            final Matrix bodyToEcefCoordinateTransformationMatrix,
            final double vx, final double vy, final double vz,
            final double x, final double y, final double z,
            final double accelerationBiasX,
            final double accelerationBiasY,
            final double accelerationBiasZ,
            final double gyroBiasX,
            final double gyroBiasY,
            final double gyroBiasZ,
            final double receiverClockOffset,
            final double receiverClockDrift,
            final Matrix covariance) {
        setBodyToEcefCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);
        setVelocityCoordinates(vx, vy, vz);
        setPositionCoordinates(x, y, z);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setReceiverClockOffset(receiverClockOffset);
        setReceiverClockDrift(receiverClockDrift);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param c                   body to ECEF coordinate transformation.
     * @param vx                  estimated ECEF user velocity resolved around x axis.
     * @param vy                  estimated ECEF user velocity resolved around y axis.
     * @param vz                  estimated ECEF user velocity resolved around z axis.
     * @param x                   x coordinate of estimated ECEF user position.
     * @param y                   y coordinate of estimated ECEF user position.
     * @param z                   z coordinate of estimated ECEF user position.
     * @param accelerationBiasX   estimated accelerometer bias resolved around x axis and
     *                            expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY   estimated accelerometer bias resolved around y axis and
     *                            expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ   estimated accelerometer bias resolved around z axis and
     *                            expressed in meters per squared second (m/s^2).
     * @param gyroBiasX           estimated gyroscope bias resolved around x axis and
     *                            expressed in radians per second (rad/s).
     * @param gyroBiasY           estimated gyroscope bias resolved around y axis and
     *                            expressed in radians per second (rad/s).
     * @param gyroBiasZ           estimated gyroscope bias resolved around z axis and
     *                            expressed in radians per second (rad/s).
     * @param receiverClockOffset estimated receiver clock offset expressed in meters (m).
     * @param receiverClockDrift  estimated receiver clock drift expressed in meters per
     *                            second (m/s).
     * @param covariance          estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided covariance matrix is not 17x17.
     */
    public INSTightlyCoupledKalmanState(
            final CoordinateTransformation c,
            final Speed vx, final Speed vy, final Speed vz,
            final Distance x, final Distance y, final Distance z,
            final double accelerationBiasX,
            final double accelerationBiasY,
            final double accelerationBiasZ,
            final double gyroBiasX,
            final double gyroBiasY,
            final double gyroBiasZ,
            final double receiverClockOffset,
            final double receiverClockDrift,
            final Matrix covariance) {
        setC(c);
        setVelocityCoordinates(vx, vy, vz);
        setPositionCoordinates(x, y, z);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setReceiverClockOffset(receiverClockOffset);
        setReceiverClockDrift(receiverClockDrift);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param c                   body to ECEF coordinate transformation.
     * @param vx                  estimated ECEF user velocity resolved around x axis.
     * @param vy                  estimated ECEF user velocity resolved around y axis.
     * @param vz                  estimated ECEF user velocity resolved around z axis.
     * @param position            estimated ECEF user position.
     * @param accelerationBiasX   estimated accelerometer bias resolved around x axis and
     *                            expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY   estimated accelerometer bias resolved around y axis and
     *                            expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ   estimated accelerometer bias resolved around z axis and
     *                            expressed in meters per squared second (m/s^2).
     * @param gyroBiasX           estimated gyroscope bias resolved around x axis and
     *                            expressed in radians per second (rad/s).
     * @param gyroBiasY           estimated gyroscope bias resolved around y axis and
     *                            expressed in radians per second (rad/s).
     * @param gyroBiasZ           estimated gyroscope bias resolved around z axis and
     *                            expressed in radians per second (rad/s).
     * @param receiverClockOffset estimated receiver clock offset expressed in meters (m).
     * @param receiverClockDrift  estimated receiver clock drift expressed in meters per
     *                            second (m/s).
     * @param covariance          estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided covariance matrix is not 17x17.
     */
    public INSTightlyCoupledKalmanState(
            final CoordinateTransformation c,
            final Speed vx, final Speed vy, final Speed vz,
            final Point3D position,
            final double accelerationBiasX,
            final double accelerationBiasY,
            final double accelerationBiasZ,
            final double gyroBiasX,
            final double gyroBiasY,
            final double gyroBiasZ,
            final double receiverClockOffset,
            final double receiverClockDrift,
            final Matrix covariance) {
        setC(c);
        setVelocityCoordinates(vx, vy, vz);
        setPosition(position);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setReceiverClockOffset(receiverClockOffset);
        setReceiverClockDrift(receiverClockDrift);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param c                   body to ECEF coordinate transformation.
     * @param velocity            estimated ECEF user velocity.
     * @param position            estimated ECEF user position.
     * @param accelerationBiasX   estimated accelerometer bias resolved around x axis and
     *                            expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY   estimated accelerometer bias resolved around y axis and
     *                            expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ   estimated accelerometer bias resolved around z axis and
     *                            expressed in meters per squared second (m/s^2).
     * @param gyroBiasX           estimated gyroscope bias resolved around x axis and
     *                            expressed in radians per second (rad/s).
     * @param gyroBiasY           estimated gyroscope bias resolved around y axis and
     *                            expressed in radians per second (rad/s).
     * @param gyroBiasZ           estimated gyroscope bias resolved around z axis and
     *                            expressed in radians per second (rad/s).
     * @param receiverClockOffset estimated receiver clock offset expressed in meters (m).
     * @param receiverClockDrift  estimated receiver clock drift expressed in meters per
     *                            second (m/s).
     * @param covariance          estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided covariance matrix is not 17x17.
     */
    public INSTightlyCoupledKalmanState(
            final CoordinateTransformation c,
            final ECEFVelocity velocity,
            final ECEFPosition position,
            final double accelerationBiasX,
            final double accelerationBiasY,
            final double accelerationBiasZ,
            final double gyroBiasX,
            final double gyroBiasY,
            final double gyroBiasZ,
            final double receiverClockOffset,
            final double receiverClockDrift,
            final Matrix covariance) {
        setC(c);
        setEcefVelocity(velocity);
        setEcefPosition(position);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setReceiverClockOffset(receiverClockOffset);
        setReceiverClockDrift(receiverClockDrift);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param c                   body to ECEF coordinate transformation.
     * @param positionAndVelocity estimated ECEF user velocity and position.
     * @param accelerationBiasX   estimated accelerometer bias resolved around x axis and
     *                            expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY   estimated accelerometer bias resolved around y axis and
     *                            expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ   estimated accelerometer bias resolved around z axis and
     *                            expressed in meters per squared second (m/s^2).
     * @param gyroBiasX           estimated gyroscope bias resolved around x axis and
     *                            expressed in radians per second (rad/s).
     * @param gyroBiasY           estimated gyroscope bias resolved around y axis and
     *                            expressed in radians per second (rad/s).
     * @param gyroBiasZ           estimated gyroscope bias resolved around z axis and
     *                            expressed in radians per second (rad/s).
     * @param receiverClockOffset estimated receiver clock offset expressed in meters (m).
     * @param receiverClockDrift  estimated receiver clock drift expressed in meters per
     *                            second (m/s).
     * @param covariance          estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided covariance matrix is not 17x17.
     */
    public INSTightlyCoupledKalmanState(
            final CoordinateTransformation c,
            final ECEFPositionAndVelocity positionAndVelocity,
            final double accelerationBiasX,
            final double accelerationBiasY,
            final double accelerationBiasZ,
            final double gyroBiasX,
            final double gyroBiasY,
            final double gyroBiasZ,
            final double receiverClockOffset,
            final double receiverClockDrift,
            final Matrix covariance) {
        setC(c);
        setPositionAndVelocity(positionAndVelocity);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setReceiverClockOffset(receiverClockOffset);
        setReceiverClockDrift(receiverClockDrift);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param frame               estimated user ECEF frame.
     * @param accelerationBiasX   estimated accelerometer bias resolved around x axis and
     *                            expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY   estimated accelerometer bias resolved around y axis and
     *                            expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ   estimated accelerometer bias resolved around z axis and
     *                            expressed in meters per squared second (m/s^2).
     * @param gyroBiasX           estimated gyroscope bias resolved around x axis and
     *                            expressed in radians per second (rad/s).
     * @param gyroBiasY           estimated gyroscope bias resolved around y axis and
     *                            expressed in radians per second (rad/s).
     * @param gyroBiasZ           estimated gyroscope bias resolved around z axis and
     *                            expressed in radians per second (rad/s).
     * @param receiverClockOffset estimated receiver clock offset expressed in meters (m).
     * @param receiverClockDrift  estimated receiver clock drift expressed in meters per
     *                            second (m/s).
     * @param covariance          estimated Kalman filter error covariance .
     * @throws IllegalArgumentException if provided covariance matrix is not 17x17.
     */
    public INSTightlyCoupledKalmanState(
            final ECEFFrame frame,
            final double accelerationBiasX,
            final double accelerationBiasY,
            final double accelerationBiasZ,
            final double gyroBiasX,
            final double gyroBiasY,
            final double gyroBiasZ,
            final double receiverClockOffset,
            final double receiverClockDrift,
            final Matrix covariance) {
        setFrame(frame);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setReceiverClockOffset(receiverClockOffset);
        setReceiverClockDrift(receiverClockDrift);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param c                   body to ECEF coordinate transformation.
     * @param vx                  estimated ECEF user velocity resolved around x axis.
     * @param vy                  estimated ECEF user velocity resolved around y axis.
     * @param vz                  estimated ECEF user velocity resolved around z axis.
     * @param x                   x coordinate of estimated ECEF user position.
     * @param y                   y coordinate of estimated ECEF user position.
     * @param z                   z coordinate of estimated ECEF user position.
     * @param accelerationBiasX   estimated accelerometer bias resolved around x axis.
     * @param accelerationBiasY   estimated accelerometer bias resolved around y axis.
     * @param accelerationBiasZ   estimated accelerometer bias resolved around z axis.
     * @param gyroBiasX           estimated gyroscope bias resolved around x axis.
     * @param gyroBiasY           estimated gyroscope bias resolved around y axis.
     * @param gyroBiasZ           estimated gyroscope bias resolved around z axis.
     * @param receiverClockOffset estimated receiver clock offset.
     * @param receiverClockDrift  estimated receiver clock drift.
     * @param covariance          estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided covariance matrix is not 17x17.
     */
    public INSTightlyCoupledKalmanState(
            final CoordinateTransformation c,
            final Speed vx, final Speed vy, final Speed vz,
            final Distance x, final Distance y, final Distance z,
            final Acceleration accelerationBiasX,
            final Acceleration accelerationBiasY,
            final Acceleration accelerationBiasZ,
            final AngularSpeed gyroBiasX,
            final AngularSpeed gyroBiasY,
            final AngularSpeed gyroBiasZ,
            final Distance receiverClockOffset,
            final Speed receiverClockDrift,
            final Matrix covariance) {
        setC(c);
        setVelocityCoordinates(vx, vy, vz);
        setPositionCoordinates(x, y, z);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setReceiverClockOffset(receiverClockOffset);
        setReceiverClockDrift(receiverClockDrift);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param c                   body to ECEF coordinate transformation.
     * @param vx                  estimated ECEF user velocity resolved around x axis.
     * @param vy                  estimated ECEF user velocity resolved around y axis.
     * @param vz                  estimated ECEF user velocity resolved around z axis.
     * @param position            estimated ECEF user position expressed in meters (m).
     * @param accelerationBiasX   estimated accelerometer bias resolved around x axis.
     * @param accelerationBiasY   estimated accelerometer bias resolved around y axis.
     * @param accelerationBiasZ   estimated accelerometer bias resolved around z axis.
     * @param gyroBiasX           estimated gyroscope bias resolved around x axis.
     * @param gyroBiasY           estimated gyroscope bias resolved around y axis.
     * @param gyroBiasZ           estimated gyroscope bias resolved around z axis.
     * @param receiverClockOffset estimated receiver clock offset.
     * @param receiverClockDrift  estimated receiver clock drift.
     * @param covariance          estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided covariance matrix is not 17x17.
     */
    public INSTightlyCoupledKalmanState(
            final CoordinateTransformation c,
            final Speed vx, final Speed vy, final Speed vz,
            final Point3D position,
            final Acceleration accelerationBiasX,
            final Acceleration accelerationBiasY,
            final Acceleration accelerationBiasZ,
            final AngularSpeed gyroBiasX,
            final AngularSpeed gyroBiasY,
            final AngularSpeed gyroBiasZ,
            final Distance receiverClockOffset,
            final Speed receiverClockDrift,
            final Matrix covariance) {
        setC(c);
        setVelocityCoordinates(vx, vy, vz);
        setPosition(position);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setReceiverClockOffset(receiverClockOffset);
        setReceiverClockDrift(receiverClockDrift);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param c                   body to ECEF coordinate transformation.
     * @param velocity            estimated ECEF user velocity.
     * @param position            estimated ECEF user position.
     * @param accelerationBiasX   estimated accelerometer bias resolved around x axis.
     * @param accelerationBiasY   estimated accelerometer bias resolved around y axis.
     * @param accelerationBiasZ   estimated accelerometer bias resolved around z axis.
     * @param gyroBiasX           estimated gyroscope bias resolved around x axis.
     * @param gyroBiasY           estimated gyroscope bias resolved around y axis.
     * @param gyroBiasZ           estimated gyroscope bias resolved around z axis.
     * @param receiverClockOffset estimated receiver clock offset.
     * @param receiverClockDrift  estimated receiver clock drift.
     * @param covariance          estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided covariance matrix is not 17x17.
     */
    public INSTightlyCoupledKalmanState(
            final CoordinateTransformation c,
            final ECEFVelocity velocity,
            final ECEFPosition position,
            final Acceleration accelerationBiasX,
            final Acceleration accelerationBiasY,
            final Acceleration accelerationBiasZ,
            final AngularSpeed gyroBiasX,
            final AngularSpeed gyroBiasY,
            final AngularSpeed gyroBiasZ,
            final Distance receiverClockOffset,
            final Speed receiverClockDrift,
            final Matrix covariance) {
        setC(c);
        setEcefVelocity(velocity);
        setEcefPosition(position);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setReceiverClockOffset(receiverClockOffset);
        setReceiverClockDrift(receiverClockDrift);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param c                   body to ECEF coordinate transformation.
     * @param positionAndVelocity estimated ECEF user position and velocity.
     * @param accelerationBiasX   estimated accelerometer bias resolved around x axis.
     * @param accelerationBiasY   estimated accelerometer bias resolved around y axis.
     * @param accelerationBiasZ   estimated accelerometer bias resolved around z axis.
     * @param gyroBiasX           estimated gyroscope bias resolved around x axis.
     * @param gyroBiasY           estimated gyroscope bias resolved around y axis.
     * @param gyroBiasZ           estimated gyroscope bias resolved around z axis.
     * @param receiverClockOffset estimated receiver clock offset.
     * @param receiverClockDrift  estimated receiver clock drift.
     * @param covariance          estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided covariance matrix is not 17x17.
     */
    public INSTightlyCoupledKalmanState(
            final CoordinateTransformation c,
            final ECEFPositionAndVelocity positionAndVelocity,
            final Acceleration accelerationBiasX,
            final Acceleration accelerationBiasY,
            final Acceleration accelerationBiasZ,
            final AngularSpeed gyroBiasX,
            final AngularSpeed gyroBiasY,
            final AngularSpeed gyroBiasZ,
            final Distance receiverClockOffset,
            final Speed receiverClockDrift,
            final Matrix covariance) {
        setC(c);
        setPositionAndVelocity(positionAndVelocity);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setReceiverClockOffset(receiverClockOffset);
        setReceiverClockDrift(receiverClockDrift);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param frame               estimated user ECEF frame.
     * @param accelerationBiasX   estimated accelerometer bias resolved around x axis.
     * @param accelerationBiasY   estimated accelerometer bias resolved around y axis.
     * @param accelerationBiasZ   estimated accelerometer bias resolved around z axis.
     * @param gyroBiasX           estimated gyroscope bias resolved around x axis.
     * @param gyroBiasY           estimated gyroscope bias resolved around y axis.
     * @param gyroBiasZ           estimated gyroscope bias resolved around z axis.
     * @param receiverClockOffset estimated receiver clock offset.
     * @param receiverClockDrift  estimated receiver clock drift.
     * @param covariance          estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided covariance matrix is not 17x17.
     */
    public INSTightlyCoupledKalmanState(
            final ECEFFrame frame,
            final Acceleration accelerationBiasX,
            final Acceleration accelerationBiasY,
            final Acceleration accelerationBiasZ,
            final AngularSpeed gyroBiasX,
            final AngularSpeed gyroBiasY,
            final AngularSpeed gyroBiasZ,
            final Distance receiverClockOffset,
            final Speed receiverClockDrift,
            final Matrix covariance) {
        setFrame(frame);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setReceiverClockOffset(receiverClockOffset);
        setReceiverClockDrift(receiverClockDrift);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param bodyToEcefCoordinateTransformationMatrix estimated body to ECEF coordinate transformation matrix.
     * @param vx                                       estimated ECEF user velocity resolved around x axis.
     * @param vy                                       estimated ECEF user velocity resolved around y axis.
     * @param vz                                       estimated ECEF user velocity resolved around z axis.
     * @param x                                        x coordinate of estimated ECEF user position.
     * @param y                                        y coordinate of estimated ECEF user position.
     * @param z                                        z coordinate of estimated ECEF user position.
     * @param accelerationBiasX                        estimated accelerometer bias resolved around x axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY                        estimated accelerometer bias resolved around y axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ                        estimated accelerometer bias resolved around z axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param gyroBiasX                                estimated gyroscope bias resolved around x axis and
     *                                                 expressed in radians per second (rad/s).
     * @param gyroBiasY                                estimated gyroscope bias resolved around y axis and
     *                                                 expressed in radians per second (rad/s).
     * @param gyroBiasZ                                estimated gyroscope bias resolved around z axis and
     *                                                 expressed in radians per second (rad/s).
     * @param receiverClockOffset                      estimated receiver clock offset expressed in meters (m).
     * @param receiverClockDrift                       estimated receiver clock drift expressed in meters per
     *                                                 second (m/s).
     * @param covariance                               estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided body to ECEF coordinate transformation matrix is not 3x3
     *                                  or if provided covariance matrix is not 17x17.
     */
    public INSTightlyCoupledKalmanState(
            final Matrix bodyToEcefCoordinateTransformationMatrix,
            final Speed vx, final Speed vy, final Speed vz,
            final Distance x, final Distance y, final Distance z,
            final double accelerationBiasX,
            final double accelerationBiasY,
            final double accelerationBiasZ,
            final double gyroBiasX,
            final double gyroBiasY,
            final double gyroBiasZ,
            final double receiverClockOffset,
            final double receiverClockDrift,
            final Matrix covariance) {
        setBodyToEcefCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);
        setVelocityCoordinates(vx, vy, vz);
        setPositionCoordinates(x, y, z);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setReceiverClockOffset(receiverClockOffset);
        setReceiverClockDrift(receiverClockDrift);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param bodyToEcefCoordinateTransformationMatrix estimated body to ECEF coordinate transformation matrix.
     * @param vx                                       estimated ECEF user velocity resolved around x axis.
     * @param vy                                       estimated ECEF user velocity resolved around y axis.
     * @param vz                                       estimated ECEF user velocity resolved around z axis.
     * @param position                                 estimated ECEF user position.
     * @param accelerationBiasX                        estimated accelerometer bias resolved around x axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY                        estimated accelerometer bias resolved around y axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ                        estimated accelerometer bias resolved around z axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param gyroBiasX                                estimated gyroscope bias resolved around x axis and
     *                                                 expressed in radians per second (rad/s).
     * @param gyroBiasY                                estimated gyroscope bias resolved around y axis and
     *                                                 expressed in radians per second (rad/s).
     * @param gyroBiasZ                                estimated gyroscope bias resolved around z axis and
     *                                                 expressed in radians per second (rad/s).
     * @param receiverClockOffset                      estimated receiver clock offset expressed in meters (m).
     * @param receiverClockDrift                       estimated receiver clock drift expressed in meters per
     *                                                 second (m/s).
     * @param covariance                               estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided body to ECEF coordinate transformation matrix is not 3x3
     *                                  or if provided covariance matrix is not 17x17.
     */
    public INSTightlyCoupledKalmanState(
            final Matrix bodyToEcefCoordinateTransformationMatrix,
            final Speed vx, final Speed vy, final Speed vz,
            final Point3D position,
            final double accelerationBiasX,
            final double accelerationBiasY,
            final double accelerationBiasZ,
            final double gyroBiasX,
            final double gyroBiasY,
            final double gyroBiasZ,
            final double receiverClockOffset,
            final double receiverClockDrift,
            final Matrix covariance) {
        setBodyToEcefCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);
        setVelocityCoordinates(vx, vy, vz);
        setPosition(position);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setReceiverClockOffset(receiverClockOffset);
        setReceiverClockDrift(receiverClockDrift);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param bodyToEcefCoordinateTransformationMatrix estimated body to ECEF coordinate transformation matrix.
     * @param velocity                                 estimated ECEF user velocity.
     * @param position                                 estimated ECEF user position.
     * @param accelerationBiasX                        estimated accelerometer bias resolved around x axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY                        estimated accelerometer bias resolved around y axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ                        estimated accelerometer bias resolved around z axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param gyroBiasX                                estimated gyroscope bias resolved around x axis and
     *                                                 expressed in radians per second (rad/s).
     * @param gyroBiasY                                estimated gyroscope bias resolved around y axis and
     *                                                 expressed in radians per second (rad/s).
     * @param gyroBiasZ                                estimated gyroscope bias resolved around z axis and
     *                                                 expressed in radians per second (rad/s).
     * @param receiverClockOffset                      estimated receiver clock offset expressed in meters (m).
     * @param receiverClockDrift                       estimated receiver clock drift expressed in meters per
     *                                                 second (m/s).
     * @param covariance                               estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided body to ECEF coordinate transformation matrix is not 3x3
     *                                  or if provided covariance matrix is not 17x17.
     */
    public INSTightlyCoupledKalmanState(
            final Matrix bodyToEcefCoordinateTransformationMatrix,
            final ECEFVelocity velocity,
            final ECEFPosition position,
            final double accelerationBiasX,
            final double accelerationBiasY,
            final double accelerationBiasZ,
            final double gyroBiasX,
            final double gyroBiasY,
            final double gyroBiasZ,
            final double receiverClockOffset,
            final double receiverClockDrift,
            final Matrix covariance) {
        setBodyToEcefCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);
        setEcefVelocity(velocity);
        setEcefPosition(position);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setReceiverClockOffset(receiverClockOffset);
        setReceiverClockDrift(receiverClockDrift);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param bodyToEcefCoordinateTransformationMatrix estimated body to ECEF coordinate transformation matrix.
     * @param positionAndVelocity                      estimated ECEF user position and velocity.
     * @param accelerationBiasX                        estimated accelerometer bias resolved around x axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY                        estimated accelerometer bias resolved around y axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ                        estimated accelerometer bias resolved around z axis and
     *                                                 expressed in meters per squared second (m/s^2).
     * @param gyroBiasX                                estimated gyroscope bias resolved around x axis and
     *                                                 expressed in radians per second (rad/s).
     * @param gyroBiasY                                estimated gyroscope bias resolved around y axis and
     *                                                 expressed in radians per second (rad/s).
     * @param gyroBiasZ                                estimated gyroscope bias resolved around z axis and
     *                                                 expressed in radians per second (rad/s).
     * @param receiverClockOffset                      estimated receiver clock offset expressed in meters (m).
     * @param receiverClockDrift                       estimated receiver clock drift expressed in meters per
     *                                                 second (m/s).
     * @param covariance                               estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided body to ECEF coordinate transformation matrix is not 3x3
     *                                  or if provided covariance matrix is not 17x17.
     */
    public INSTightlyCoupledKalmanState(
            final Matrix bodyToEcefCoordinateTransformationMatrix,
            final ECEFPositionAndVelocity positionAndVelocity,
            final double accelerationBiasX,
            final double accelerationBiasY,
            final double accelerationBiasZ,
            final double gyroBiasX,
            final double gyroBiasY,
            final double gyroBiasZ,
            final double receiverClockOffset,
            final double receiverClockDrift,
            final Matrix covariance) {
        setBodyToEcefCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);
        setPositionAndVelocity(positionAndVelocity);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setReceiverClockOffset(receiverClockOffset);
        setReceiverClockDrift(receiverClockDrift);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param bodyToEcefCoordinateTransformationMatrix estimated body to ECEF coordinate transformation matrix.
     * @param vx                                       estimated ECEF user velocity resolved around x axis.
     * @param vy                                       estimated ECEF user velocity resolved around y axis.
     * @param vz                                       estimated ECEF user velocity resolved around z axis.
     * @param x                                        x coordinate of estimated ECEF user position.
     * @param y                                        y coordinate of estimated ECEF user position.
     * @param z                                        z coordinate of estimated ECEF user position.
     * @param accelerationBiasX                        estimated accelerometer bias resolved around x axis.
     * @param accelerationBiasY                        estimated accelerometer bias resolved around y axis.
     * @param accelerationBiasZ                        estimated accelerometer bias resolved around z axis.
     * @param gyroBiasX                                estimated gyroscope bias resolved around x axis.
     * @param gyroBiasY                                estimated gyroscope bias resolved around y axis.
     * @param gyroBiasZ                                estimated gyroscope bias resolved around z axis.
     * @param receiverClockOffset                      estimated receiver clock offset.
     * @param receiverClockDrift                       estimated receiver clock drift.
     * @param covariance                               estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided body to ECEF coordinate transformation matrix is not 3x3
     *                                  or if provided covariance matrix is not 17x17.
     */
    public INSTightlyCoupledKalmanState(
            final Matrix bodyToEcefCoordinateTransformationMatrix,
            final Speed vx, final Speed vy, final Speed vz,
            final Distance x, final Distance y, final Distance z,
            final Acceleration accelerationBiasX,
            final Acceleration accelerationBiasY,
            final Acceleration accelerationBiasZ,
            final AngularSpeed gyroBiasX,
            final AngularSpeed gyroBiasY,
            final AngularSpeed gyroBiasZ,
            final Distance receiverClockOffset,
            final Speed receiverClockDrift,
            final Matrix covariance) {
        setBodyToEcefCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);
        setVelocityCoordinates(vx, vy, vz);
        setPositionCoordinates(x, y, z);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setReceiverClockOffset(receiverClockOffset);
        setReceiverClockDrift(receiverClockDrift);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param bodyToEcefCoordinateTransformationMatrix estimated body to ECEF coordinate transformation matrix.
     * @param vx                                       estimated ECEF user velocity resolved around x axis.
     * @param vy                                       estimated ECEF user velocity resolved around y axis.
     * @param vz                                       estimated ECEF user velocity resolved around z axis.
     * @param position                                 estimated ECEF user position expressed in meters (m).
     * @param accelerationBiasX                        estimated accelerometer bias resolved around x axis.
     * @param accelerationBiasY                        estimated accelerometer bias resolved around y axis.
     * @param accelerationBiasZ                        estimated accelerometer bias resolved around z axis.
     * @param gyroBiasX                                estimated gyroscope bias resolved around x axis.
     * @param gyroBiasY                                estimated gyroscope bias resolved around y axis.
     * @param gyroBiasZ                                estimated gyroscope bias resolved around z axis.
     * @param receiverClockOffset                      estimated receiver clock offset.
     * @param receiverClockDrift                       estimated receiver clock drift.
     * @param covariance                               estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided body to ECEF coordinate transformation matrix is not 3x3
     *                                  or if provided covariance matrix is not 17x17.
     */
    public INSTightlyCoupledKalmanState(
            final Matrix bodyToEcefCoordinateTransformationMatrix,
            final Speed vx, final Speed vy, final Speed vz,
            final Point3D position,
            final Acceleration accelerationBiasX,
            final Acceleration accelerationBiasY,
            final Acceleration accelerationBiasZ,
            final AngularSpeed gyroBiasX,
            final AngularSpeed gyroBiasY,
            final AngularSpeed gyroBiasZ,
            final Distance receiverClockOffset,
            final Speed receiverClockDrift,
            final Matrix covariance) {
        setBodyToEcefCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);
        setVelocityCoordinates(vx, vy, vz);
        setPosition(position);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setReceiverClockOffset(receiverClockOffset);
        setReceiverClockDrift(receiverClockDrift);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param bodyToEcefCoordinateTransformationMatrix estimated body to ECEF coordinate transformation matrix.
     * @param velocity                                 estimated ECEF user velocity.
     * @param position                                 estimated ECEF user position.
     * @param accelerationBiasX                        estimated accelerometer bias resolved around x axis.
     * @param accelerationBiasY                        estimated accelerometer bias resolved around y axis.
     * @param accelerationBiasZ                        estimated accelerometer bias resolved around z axis.
     * @param gyroBiasX                                estimated gyroscope bias resolved around x axis.
     * @param gyroBiasY                                estimated gyroscope bias resolved around y axis.
     * @param gyroBiasZ                                estimated gyroscope bias resolved around z axis.
     * @param receiverClockOffset                      estimated receiver clock offset.
     * @param receiverClockDrift                       estimated receiver clock drift.
     * @param covariance                               estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided body to ECEF coordinate transformation matrix is not 3x3
     *                                  or if provided covariance matrix is not 17x17.
     */
    public INSTightlyCoupledKalmanState(
            final Matrix bodyToEcefCoordinateTransformationMatrix,
            final ECEFVelocity velocity,
            final ECEFPosition position,
            final Acceleration accelerationBiasX,
            final Acceleration accelerationBiasY,
            final Acceleration accelerationBiasZ,
            final AngularSpeed gyroBiasX,
            final AngularSpeed gyroBiasY,
            final AngularSpeed gyroBiasZ,
            final Distance receiverClockOffset,
            final Speed receiverClockDrift,
            final Matrix covariance) {
        setBodyToEcefCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);
        setEcefVelocity(velocity);
        setEcefPosition(position);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setReceiverClockOffset(receiverClockOffset);
        setReceiverClockDrift(receiverClockDrift);
        setCovariance(covariance);
    }

    /**
     * Constructor.
     *
     * @param bodyToEcefCoordinateTransformationMatrix estimated body to ECEF coordinate transformation matrix.
     * @param positionAndVelocity                      estimated ECEF user position and velocity.
     * @param accelerationBiasX                        estimated accelerometer bias resolved around x axis.
     * @param accelerationBiasY                        estimated accelerometer bias resolved around y axis.
     * @param accelerationBiasZ                        estimated accelerometer bias resolved around z axis.
     * @param gyroBiasX                                estimated gyroscope bias resolved around x axis.
     * @param gyroBiasY                                estimated gyroscope bias resolved around y axis.
     * @param gyroBiasZ                                estimated gyroscope bias resolved around z axis.
     * @param receiverClockOffset                      estimated receiver clock offset.
     * @param receiverClockDrift                       estimated receiver clock drift.
     * @param covariance                               estimated Kalman filter error covariance matrix.
     * @throws IllegalArgumentException if provided body to ECEF coordinate transformation matrix is not 3x3
     *                                  or if provided covariance matrix is not 17x17.
     */
    public INSTightlyCoupledKalmanState(
            final Matrix bodyToEcefCoordinateTransformationMatrix,
            final ECEFPositionAndVelocity positionAndVelocity,
            final Acceleration accelerationBiasX,
            final Acceleration accelerationBiasY,
            final Acceleration accelerationBiasZ,
            final AngularSpeed gyroBiasX,
            final AngularSpeed gyroBiasY,
            final AngularSpeed gyroBiasZ,
            final Distance receiverClockOffset,
            final Speed receiverClockDrift,
            final Matrix covariance) {
        setBodyToEcefCoordinateTransformationMatrix(bodyToEcefCoordinateTransformationMatrix);
        setPositionAndVelocity(positionAndVelocity);
        setAccelerationBiasCoordinates(accelerationBiasX, accelerationBiasY, accelerationBiasZ);
        setGyroBiasCoordinates(gyroBiasX, gyroBiasY, gyroBiasZ);
        setReceiverClockOffset(receiverClockOffset);
        setReceiverClockDrift(receiverClockDrift);
        setCovariance(covariance);
    }

    /**
     * Copy constructor.
     *
     * @param input input instance to copy data from.
     */
    public INSTightlyCoupledKalmanState(final INSTightlyCoupledKalmanState input) {
        copyFrom(input);
    }

    /**
     * Gets estimated body to ECEF coordinate transformation matrix.
     *
     * @return estimated body to ECEF coordinate transformation matrix.
     */
    public Matrix getBodyToEcefCoordinateTransformationMatrix() {
        return mBodyToEcefCoordinateTransformationMatrix;
    }

    /**
     * Sets estimated body to ECEF coordinate transformation matrix.
     *
     * @param bodyToEcefCoordinateTransformationMatrix estimated body to ECEF coordinate
     *                                                 transformation matrix.
     * @throws IllegalArgumentException if provided matrix is not 3x3.
     */
    public void setBodyToEcefCoordinateTransformationMatrix(
            final Matrix bodyToEcefCoordinateTransformationMatrix) {
        if (bodyToEcefCoordinateTransformationMatrix.getRows() != CoordinateTransformation.ROWS ||
                bodyToEcefCoordinateTransformationMatrix.getColumns() != CoordinateTransformation.COLS) {
            throw new IllegalArgumentException();
        }
        mBodyToEcefCoordinateTransformationMatrix =
                bodyToEcefCoordinateTransformationMatrix;
    }

    /**
     * Gets estimated ECEF user velocity resolved around x axis and expressed in meters per second (m/s).
     *
     * @return estimated ECEF user velocity resolved around x axis and expressed in meters per second (m/s).
     */
    public double getVx() {
        return mVx;
    }

    /**
     * Sets estimated ECEF user velocity resolved around x axis and expressed in meters per second (m/s).
     *
     * @param vx estimated ECEF user velocity resolved around x axis and expressed in meters per second (m/s).
     */
    public void setVx(final double vx) {
        mVx = vx;
    }

    /**
     * Gets estimated ECEF user velocity resolved around y axis and expressed in meters per second (m/s).
     *
     * @return estimated ECEF user velocity resolved around y axis and expressed in meters per second (m/s).
     */
    public double getVy() {
        return mVy;
    }

    /**
     * Sets estimated ECEF user velocity resolved around y axis and expressed in meters per second (m/s).
     *
     * @param vy estimated ECEF user velocity resolved around y axis and expressed in meters per scond (m/s).
     */
    public void setVy(final double vy) {
        mVy = vy;
    }

    /**
     * Gets estimated ECEF user velocity resolved around z axis and expressed in meters per second (m/s).
     *
     * @return estimated ECEF user velocity resolved around z axis and expressed in meters per second (m/s).
     */
    public double getVz() {
        return mVz;
    }

    /**
     * Sets estimated ECEF user velocity resolved around z axis and expressed in meters per second (m/s).
     *
     * @param vz estimated ECEF user velocity resolved around z axis and expressed in meters per second (m/s).
     */
    public void setVz(final double vz) {
        mVz = vz;
    }

    /**
     * Sets estimated ECEF user velocity coordinates.
     *
     * @param vx estimated ECEF user velocity resolved around x axis and expressed in meters per second (m/s).
     * @param vy estimated ECEF user velocity resolved around y axis and expressed in meters per second (m/s).
     * @param vz estimated ECEF user velocity resolved around z axis and expressed in meters per second (m/s).
     */
    public void setVelocityCoordinates(
            final double vx, final double vy, final double vz) {
        mVx = vx;
        mVy = vy;
        mVz = vz;
    }

    /**
     * Gets x coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @return x coordinate of estimated ECEF user position expressed in meters (m).
     */
    public double getX() {
        return mX;
    }

    /**
     * Sets x coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @param x x coordinate of estimated ECEF user position expressed in meters (m).
     */
    public void setX(final double x) {
        mX = x;
    }

    /**
     * Gets y coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @return y coordinate of estimated ECEF user position expressed in meters (m).
     */
    public double getY() {
        return mY;
    }

    /**
     * Sets y coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @param y y coordinate of estimated ECEF user position expressed in meters (m).
     */
    public void setY(final double y) {
        mY = y;
    }

    /**
     * Gets z coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @return z coordinate of estimated ECEF user position expressed in meters (m).
     */
    public double getZ() {
        return mZ;
    }

    /**
     * Sets z coordinate of estimated ECEF user position expressed in meters (m).
     *
     * @param z z coordinate of estimated ECEF user position expressed in meters (m).
     */
    public void setZ(final double z) {
        mZ = z;
    }

    /**
     * Sets estimated ECEF user position coordinates.
     *
     * @param x x coordinate of estimated ECEF user position expressed in meters (m).
     * @param y y coordinate of estimated ECEF user position expressed in meters (m).
     * @param z z coordinate of estimated ECEF user position expressed in meters (m).
     */
    public void setPositionCoordinates(
            final double x, final double y, final double z) {
        mX = x;
        mY = y;
        mZ = z;
    }

    /**
     * Gets estimated accelerometer bias resolved around x axis and expressed in
     * meters per squared second (m/s^2).
     *
     * @return estimated accelerometer bias resolved around x axis and expressed in
     * meters per squared second (m/s^2).
     */
    public double getAccelerationBiasX() {
        return mAccelerationBiasX;
    }

    /**
     * Sets estimated accelerometer bias resolved around x axis and expressed in
     * meters per squared second (m/s^2).
     *
     * @param accelerationBiasX estimated accelerometer bias resolved around x axis and
     *                          expressed in meters per squared second (m/s^2).
     */
    public void setAccelerationBiasX(final double accelerationBiasX) {
        mAccelerationBiasX = accelerationBiasX;
    }

    /**
     * Gets estimated accelerometer bias resolved around y axis and expressed in
     * meters per squared second (m/s^2).
     *
     * @return estimated accelerometer bias resolved around y axis and expressed in
     * meters per squared second (m/s^2).
     */
    public double getAccelerationBiasY() {
        return mAccelerationBiasY;
    }

    /**
     * Sets estimated accelerometer bias resolved around y axis and expressed in
     * meters per squared second (m/s^2).
     *
     * @param accelerationBiasY estimated accelerometer bias resolved around y axis
     *                          and expressed in meters per squared second (m/s^2).
     */
    public void setAccelerationBiasY(final double accelerationBiasY) {
        mAccelerationBiasY = accelerationBiasY;
    }

    /**
     * Gets estimated accelerometer bias resolved around z axis and expressed in
     * meters per squared second (m/s^2).
     *
     * @return estimated accelerometer bias resolved around z axis and
     * expressed in meters per squared second (m/s^2).
     */
    public double getAccelerationBiasZ() {
        return mAccelerationBiasZ;
    }

    /**
     * Sets estimated accelerometer bias resolved around z axis and expressed in
     * meters per squared second (m/s^2).
     *
     * @param accelerationBiasZ estimated accelerometer bias resolved around z axis
     *                          and expressed in meters per squared second (m/s^2).
     */
    public void setAccelerationBiasZ(final double accelerationBiasZ) {
        mAccelerationBiasZ = accelerationBiasZ;
    }

    /**
     * Sets estimated accelerometer bias expressed in meters per squared second (m/s^2).
     *
     * @param accelerationBiasX estimated accelerometer bias resolved around x axis and
     *                          expressed in meters per squared second (m/s^2).
     * @param accelerationBiasY estimated accelerometer bias resolved around y axis and
     *                          expressed in meters per squared second (m/s^2).
     * @param accelerationBiasZ estimated accelerometer bias resolved around z axis and
     *                          expressed in meters per squared second (m/s^2).
     */
    public void setAccelerationBiasCoordinates(
            final double accelerationBiasX, final double accelerationBiasY,
            final double accelerationBiasZ) {
        mAccelerationBiasX = accelerationBiasX;
        mAccelerationBiasY = accelerationBiasY;
        mAccelerationBiasZ = accelerationBiasZ;
    }

    /**
     * Gets estimated gyroscope bias resolved around x axis and expressed in
     * radians per second (rad/s).
     *
     * @return estimated gyroscope bias resolved around x axis and expressed in
     * radians per second (rad/s).
     */
    public double getGyroBiasX() {
        return mGyroBiasX;
    }

    /**
     * Sets estimated gyroscope bias resolved around x axis and expressed in
     * radians per second (rad/s).
     *
     * @param gyroBiasX estimated gyroscope bias resolved around x axis and
     *                  expressed in radians per second (rad/s).
     */
    public void setGyroBiasX(final double gyroBiasX) {
        mGyroBiasX = gyroBiasX;
    }

    /**
     * Gets estimated gyroscope bias resolved around y axis and expressed in
     * radians per second (rad/s).
     *
     * @return estimated gyroscope bias resolved around y axis and expressed
     * in radians per second (rad/s).
     */
    public double getGyroBiasY() {
        return mGyroBiasY;
    }

    /**
     * Sets estimated gyroscope bias resolved around y axis and expressed in
     * radians per second (rad/s).
     *
     * @param gyroBiasY estimated gyroscope bias resolved around y axis and
     *                  expressed in radians per second (rad/s).
     */
    public void setGyroBiasY(final double gyroBiasY) {
        mGyroBiasY = gyroBiasY;
    }

    /**
     * Gets estimated gyroscope bias resolved around z axis and expressed in
     * radians per second (rad/s).
     *
     * @return estimated gyroscope bias resolved around z axis and expressed
     * in radians per second (rad/s).
     */
    public double getGyroBiasZ() {
        return mGyroBiasZ;
    }

    /**
     * Sets estimated gyroscope bias resolved around z axis and expressed in
     * radians per second (rad/s).
     *
     * @param gyroBiasZ estimated gyroscope bias resolved around z axis and
     *                  expressed in radians per second (rad/s).
     */
    public void setGyroBiasZ(final double gyroBiasZ) {
        mGyroBiasZ = gyroBiasZ;
    }

    /**
     * Sets estimated gyroscope bias coordinates expressed in radians
     * per second (rad/s).
     *
     * @param gyroBiasX estimated gyroscope bias resolved around x axis and
     *                  expressed in radians per second (rad/s).
     * @param gyroBiasY estimated gyroscope bias resolved around y axis and
     *                  expressed in radians per second (rad/s).
     * @param gyroBiasZ estimated gyroscope bias resolved around z axis and
     *                  expressed in radians per second (rad/s).
     */
    public void setGyroBiasCoordinates(
            final double gyroBiasX, final double gyroBiasY, final double gyroBiasZ) {
        mGyroBiasX = gyroBiasX;
        mGyroBiasY = gyroBiasY;
        mGyroBiasZ = gyroBiasZ;
    }

    /**
     * Gets estimated receiver clock offset expressed in meters (m).
     *
     * @return estimated receiver clock offset expressed in meters (m).
     */
    public double getReceiverClockOffset() {
        return mReceiverClockOffset;
    }

    /**
     * Sets estimated receiver clock offset expressed in meters (m).
     *
     * @param receiverClockOffset estimated receiver clock offset expressed
     *                            in meters (m).
     */
    public void setReceiverClockOffset(final double receiverClockOffset) {
        mReceiverClockOffset = receiverClockOffset;
    }

    /**
     * Gets estimated receiver clock drift expressed in meters per second (m/s).
     *
     * @return estimated receiver clock drift expressed in meters per second (m/s).
     */
    public double getReceiverClockDrift() {
        return mReceiverClockDrift;
    }

    /**
     * Sets estimated receiver clock drift expressed in meters per second (m/s).
     *
     * @param receiverClockDrift estimated receiver clock drift expressed in
     *                           meters per second (m/s).
     */
    public void setReceiverClockDrift(final double receiverClockDrift) {
        mReceiverClockDrift = receiverClockDrift;
    }

    /**
     * Gets Kalman filter error covariance matrix.
     *
     * @param result instance where result data will be copied to.
     * @return true if result data has been copied, false otherwise.
     */
    public boolean getCovariance(final Matrix result) {
        if (mCovariance != null) {
            mCovariance.copyTo(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets Kalman filter error covariance matrix.
     *
     * @return Kalman filter error covariance matrix.
     */
    public Matrix getCovariance() {
        return mCovariance;
    }

    /**
     * Sets Kalman filter error covariance matrix.
     *
     * @param covariance Kalman filter error covariance matrix to be set.
     * @throws IllegalArgumentException if provided covariance matrix is not 17x17.
     */
    public void setCovariance(final Matrix covariance) {
        if (covariance.getRows() != NUM_PARAMS ||
                covariance.getColumns() != NUM_PARAMS) {
            throw new IllegalArgumentException();
        }

        mCovariance = covariance;
    }

    /**
     * Gets body to ECEF coordinate transformation.
     *
     * @return body to ECEF coordinate transformation.
     * @throws InvalidRotationMatrixException if current body to ECEF transformation matrix is
     *                                        not valid (is not a 3x3 orthonormal matrix).
     */
    public CoordinateTransformation getC() throws InvalidRotationMatrixException {
        return mBodyToEcefCoordinateTransformationMatrix != null ?
                new CoordinateTransformation(mBodyToEcefCoordinateTransformationMatrix,
                        FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME) : null;
    }

    /**
     * Gets body to ECEF coordinate transformation.
     *
     * @param threshold threshold to determine whether current body to ECEF transformation
     *                  matrix is valid or not (to check that matrix is 3x3 orthonormal).
     * @return body to ECEF coordinate transformation.
     * @throws InvalidRotationMatrixException if current body to ECEF transformation matrix
     *                                        is considered not valid (is not a 3x3 orthonormal matrix) with provided threshold.
     */
    public CoordinateTransformation getC(final double threshold) throws InvalidRotationMatrixException {
        return mBodyToEcefCoordinateTransformationMatrix != null ?
                new CoordinateTransformation(mBodyToEcefCoordinateTransformationMatrix,
                        FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME, threshold) : null;
    }

    /**
     * Gets body to ECEF coordinate transformation.
     *
     * @param result instance where body to ECEF coordinate transformation will be stored.
     * @return true if result instance was updated, false otherwise.
     * @throws InvalidRotationMatrixException if current body to ECEF transformation matrix
     *                                        is not valid (is not a 3x3 orthonormal matrix).
     */
    public boolean getC(final CoordinateTransformation result) throws InvalidRotationMatrixException {
        if (mBodyToEcefCoordinateTransformationMatrix != null) {
            result.setSourceType(FrameType.BODY_FRAME);
            result.setDestinationType(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            result.setMatrix(mBodyToEcefCoordinateTransformationMatrix);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets body to ECEF coordinate transformation.
     *
     * @param result    instance where body to ECEF coordinate transformation will be stored.
     * @param threshold threshold to determine whether current body to ECEF transformation
     *                  matrix is valid or not (to check that matrix is 3x3 orthonormal).
     * @return true if result instance was updated, false otherwise.
     * @throws InvalidRotationMatrixException if current body to ECEF transformation matrix
     *                                        is not valid (is not a 3x3 orthonormal matrix) with provided threshold.
     */
    public boolean getC(final CoordinateTransformation result,
                        final double threshold) throws InvalidRotationMatrixException {
        if (mBodyToEcefCoordinateTransformationMatrix != null) {
            result.setSourceType(FrameType.BODY_FRAME);
            result.setDestinationType(FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
            result.setMatrix(mBodyToEcefCoordinateTransformationMatrix, threshold);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets body to ECEF coordinate transformation.
     *
     * @param c body to ECEF coordinate transformation to be set.
     * @throws IllegalArgumentException if provided coordinate transformation is
     *                                  not null and is not a body to ECEF transformation.
     */
    public void setC(final CoordinateTransformation c) {
        if (c == null) {
            mBodyToEcefCoordinateTransformationMatrix = null;

        } else {

            if (c.getSourceType() != FrameType.BODY_FRAME ||
                    c.getDestinationType() != FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME) {
                throw new IllegalArgumentException();
            }

            if (mBodyToEcefCoordinateTransformationMatrix != null) {
                c.getMatrix(mBodyToEcefCoordinateTransformationMatrix);
            } else {
                mBodyToEcefCoordinateTransformationMatrix = c.getMatrix();
            }
        }
    }

    /**
     * Gets estimated ECEF user velocity resolved around x axis.
     *
     * @param result instance where estimated ECEF user velocity resolved around x axis will be stored.
     */
    public void getSpeedX(final Speed result) {
        result.setValue(mVx);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets estimated ECEF user velocity resolved around x axis.
     *
     * @return estimated ECEF user velocity resolved around x axis.
     */
    public Speed getSpeedX() {
        return new Speed(mVx, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets estimated ECEF user velocity resolved around x axis.
     *
     * @param vx estimated ECEF user velocity resolved around x axis.
     */
    public void setSpeedX(final Speed vx) {
        mVx = SpeedConverter.convert(vx.getValue().doubleValue(),
                vx.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets estimated ECEF user velocity resolved around y axis.
     *
     * @param result instance where estimated ECEF user velocity resolved around y axis will be stored.
     */
    public void getSpeedY(final Speed result) {
        result.setValue(mVy);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets estimated ECEF user velocity resolved around y axis.
     *
     * @return estimated ECEF velocity resolved around y axis.
     */
    public Speed getSpeedY() {
        return new Speed(mVy, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets estimated ECEF user velocity resolved around y axis.
     *
     * @param vy estimated ECEF user velocity resolved around y axis.
     */
    public void setSpeedY(final Speed vy) {
        mVy = SpeedConverter.convert(vy.getValue().doubleValue(),
                vy.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets estimated ECEF user velocity resolved around z axis.
     *
     * @param result instance where estimated ECEF user velocity resolved around z axis will be stored.
     */
    public void getSpeedZ(final Speed result) {
        result.setValue(mVz);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets estimated ECEF user velocity resolved around z axis.
     *
     * @return estimated ECEF velocity resolved around z axis.
     */
    public Speed getSpeedZ() {
        return new Speed(mVz, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets estimated ECEF user velocity resolved around z axis.
     *
     * @param vz estimated ECEF velocity resolved around z axis.
     */
    public void setSpeedZ(final Speed vz) {
        mVz = SpeedConverter.convert(vz.getValue().doubleValue(),
                vz.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets estimated ECEF user velocity.
     *
     * @param vx estimated ECEF velocity resolved around x axis.
     * @param vy estimated ECEF velocity resolved around y axis.
     * @param vz estimated ECEF velocity resolved around z axis.
     */
    public void setVelocityCoordinates(
            final Speed vx, final Speed vy, final Speed vz) {
        setSpeedX(vx);
        setSpeedY(vy);
        setSpeedZ(vz);
    }

    /**
     * Gets estimated ECEF user velocity.
     *
     * @param result instance where estimated ECEF user velocity will be stored.
     */
    public void getEcefVelocity(final ECEFVelocity result) {
        result.setCoordinates(mVx, mVy, mVz);
    }

    /**
     * Gets estimated ECEF user velocity.
     *
     * @return estimated ECEF user velocity.
     */
    public ECEFVelocity getEcefVelocity() {
        return new ECEFVelocity(mVx, mVy, mVz);
    }

    /**
     * Sets estimated ECEF user velocity.
     *
     * @param ecefVelocity estimated ECEF user velocity.
     */
    public void setEcefVelocity(final ECEFVelocity ecefVelocity) {
        mVx = ecefVelocity.getVx();
        mVy = ecefVelocity.getVy();
        mVz = ecefVelocity.getVz();
    }

    /**
     * Gets x coordinate of estimated ECEF user position.
     *
     * @param result instance where x coordinate of estimated ECEF user position
     *               will be stored.
     */
    public void getDistanceX(final Distance result) {
        result.setValue(mX);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets x coordinate of estimated ECEF user position.
     *
     * @return x coordinate of estimated ECEF user position.
     */
    public Distance getDistanceX() {
        return new Distance(mX, DistanceUnit.METER);
    }

    /**
     * Sets x coordinate of estimated ECEF user position.
     *
     * @param x x coordinate of estimated ECEF user position.
     */
    public void setDistanceX(final Distance x) {
        mX = DistanceConverter.convert(x.getValue().doubleValue(),
                x.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets y coordinate of estimated ECEF user position.
     *
     * @param result instance where y coordinate of estimated ECEF user position
     *               will be stored.
     */
    public void getDistanceY(final Distance result) {
        result.setValue(mY);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets y coordinate of estimated ECEF user position.
     *
     * @return y coordinate of estimated ECEF user position.
     */
    public Distance getDistanceY() {
        return new Distance(mY, DistanceUnit.METER);
    }

    /**
     * Sets y coordinate of estimated ECEF user position.
     *
     * @param y y coordinate of estimated ECEF user position.
     */
    public void setDistanceY(final Distance y) {
        mY = DistanceConverter.convert(y.getValue().doubleValue(),
                y.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets z coordinate of estimated ECEF user position.
     *
     * @param result instance where z coordinate of estimated ECEF user position
     *               will be stored.
     */
    public void getDistanceZ(final Distance result) {
        result.setValue(mZ);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets z coordinate of estimated ECEF user position.
     *
     * @return z coordinate of estimated ECEF user position.
     */
    public Distance getDistanceZ() {
        return new Distance(mZ, DistanceUnit.METER);
    }

    /**
     * Sets z coordinate of estimated ECEF user position.
     *
     * @param z z coordinate of estimated ECEF user position.
     */
    public void setDistanceZ(final Distance z) {
        mZ = DistanceConverter.convert(z.getValue().doubleValue(),
                z.getUnit(), DistanceUnit.METER);
    }

    /**
     * Sets coordinates of estimated ECEF user position.
     *
     * @param x x coordinate of estimated ECEF user position.
     * @param y y coordinate of estimated ECEF user position.
     * @param z z coordinate of estimated ECEF user position.
     */
    public void setPositionCoordinates(final Distance x, final Distance y, final Distance z) {
        setDistanceX(x);
        setDistanceY(y);
        setDistanceZ(z);
    }

    /**
     * Gets estimated ECEF user position expressed in meters (m).
     *
     * @param result instance where estimated ECEF user position expressed
     *               in meters (m) will be stored.
     */
    public void getPosition(final Point3D result) {
        result.setInhomogeneousCoordinates(mX, mY, mZ);
    }

    /**
     * Gets estimated ECEF user position expressed in meters (m).
     *
     * @return estimated ECEF user position expressed in meters (m).
     */
    public Point3D getPosition() {
        return new InhomogeneousPoint3D(mX, mY, mZ);
    }

    /**
     * Sets estimated ECEF user position expressed in meters (m).
     *
     * @param position estimated ECEF user position expressed in
     *                 meters (m).
     */
    public void setPosition(final Point3D position) {
        mX = position.getInhomX();
        mY = position.getInhomY();
        mZ = position.getInhomZ();
    }

    /**
     * Gets estimated ECEF user position.
     *
     * @param result instance where estimated ECEF user position
     *               will be stored.
     */
    public void getEcefPosition(final ECEFPosition result) {
        result.setCoordinates(mX, mY, mZ);
    }

    /**
     * Gets estimated ECEF user position.
     *
     * @return estimated ECEF user position.
     */
    public ECEFPosition getEcefPosition() {
        return new ECEFPosition(mX, mY, mZ);
    }

    /**
     * Sets estimated ECEF user position.
     *
     * @param ecefPosition estimated ECEF user position.
     */
    public void setEcefPosition(final ECEFPosition ecefPosition) {
        mX = ecefPosition.getX();
        mY = ecefPosition.getY();
        mZ = ecefPosition.getZ();
    }

    /**
     * Gets estimated ECEF user position and velocity.
     *
     * @param result instance where estimated ECEF user position and velocity
     *               will be stored.
     */
    public void getPositionAndVelocity(final ECEFPositionAndVelocity result) {
        result.setPositionCoordinates(mX, mY, mZ);
        result.setVelocityCoordinates(mVx, mVy, mVz);
    }

    /**
     * Gets estimated ECEF user position and velocity.
     *
     * @return estimated ECEF user position and velocity.
     */
    public ECEFPositionAndVelocity getPositionAndVelocity() {
        return new ECEFPositionAndVelocity(mX, mY, mZ, mVx, mVy, mVz);
    }

    /**
     * Sets estimated ECEF user position and velocity.
     *
     * @param positionAndVelocity estimated ECEF user position and velocity.
     */
    public void setPositionAndVelocity(final ECEFPositionAndVelocity positionAndVelocity) {
        mX = positionAndVelocity.getX();
        mY = positionAndVelocity.getY();
        mZ = positionAndVelocity.getZ();
        mVx = positionAndVelocity.getVx();
        mVy = positionAndVelocity.getVy();
        mVz = positionAndVelocity.getVz();
    }

    /**
     * Gets body to ECEF frame containing coordinate transformation, position and
     * velocity.
     *
     * @param result instance where body to ECEF frame will be stored.
     * @return true if result was updated, false otherwise.
     */
    public boolean getFrame(ECEFFrame result) {
        if (mBodyToEcefCoordinateTransformationMatrix != null) {
            try {
                result.setCoordinateTransformation(getC());
            } catch (final InvalidSourceAndDestinationFrameTypeException
                    | InvalidRotationMatrixException e) {
                return false;
            }
            result.setCoordinates(mX, mY, mZ);
            result.setVelocityCoordinates(mVx, mVy, mVz);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets body to ECEF frame containing coordinate transformation, position and
     * velocity.
     *
     * @return body to ECEF frame.
     */
    public ECEFFrame getFrame() {
        if (mBodyToEcefCoordinateTransformationMatrix != null) {
            try {
                return new ECEFFrame(mX, mY, mZ, mVx, mVy, mVz, getC());
            } catch (final InvalidSourceAndDestinationFrameTypeException
                    | InvalidRotationMatrixException e) {
                return null;
            }
        } else {
            return null;
        }
    }

    /**
     * Sets body to ECEF frame containing coordinate transformation, position and
     * velocity.
     *
     * @param frame body to ECEF frame to be set.
     */
    public void setFrame(final ECEFFrame frame) {
        mX = frame.getX();
        mY = frame.getY();
        mZ = frame.getZ();

        mVx = frame.getVx();
        mVy = frame.getVy();
        mVz = frame.getVz();

        if (mBodyToEcefCoordinateTransformationMatrix != null) {
            frame.getCoordinateTransformationMatrix(mBodyToEcefCoordinateTransformationMatrix);
        } else {
            mBodyToEcefCoordinateTransformationMatrix = frame.getCoordinateTransformationMatrix();
        }
    }

    /**
     * Gets estimated accelerometer bias resolved around x axis.
     *
     * @param result instance where estimated accelerometer bias resolved around
     *               x axis will be stored.
     */
    public void getAccelerationBiasXAsAcceleration(final Acceleration result) {
        result.setValue(mAccelerationBiasX);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated accelerometer bias resolved around x axis.
     *
     * @return estimated accelerometer bias resolved around x axis.
     */
    public Acceleration getAccelerationBiasXAsAcceleration() {
        return new Acceleration(mAccelerationBiasX,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets estimated accelerometer bias resolved around x axis.
     *
     * @param accelerationBiasX estimated accelerometer bias resolved
     *                          around x axis.
     */
    public void setAccelerationBiasX(final Acceleration accelerationBiasX) {
        mAccelerationBiasX = AccelerationConverter.convert(
                accelerationBiasX.getValue().doubleValue(),
                accelerationBiasX.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated accelerometer bias resolved around y axis.
     *
     * @param result instance where estimated accelerometer bias resolved around
     *               y axis will be stored.
     */
    public void getAccelerationBiasYAsAcceleration(final Acceleration result) {
        result.setValue(mAccelerationBiasY);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated accelerometer bias resolved around y axis.
     *
     * @return estimated accelerometer bias resolved around y axis.
     */
    public Acceleration getAccelerationBiasYAsAcceleration() {
        return new Acceleration(mAccelerationBiasY,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets estimated accelerometer bias resolved around y axis.
     *
     * @param accelerationBiasY estimated accelerometer bias resolved
     *                          around y axis.
     */
    public void setAccelerationBiasY(final Acceleration accelerationBiasY) {
        mAccelerationBiasY = AccelerationConverter.convert(
                accelerationBiasY.getValue().doubleValue(),
                accelerationBiasY.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated accelerometer bias resolved around z axis.
     *
     * @param result instance where estimated accelerometer bias resolved around
     *               z axis will be stored.
     */
    public void getAccelerationBiasZAsAcceleration(final Acceleration result) {
        result.setValue(mAccelerationBiasZ);
        result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Gets estimated accelerometer bias resolved around z axis.
     *
     * @return estimated accelerometer bias resolved around z axis.
     */
    public Acceleration getAccelerationBiasZAsAcceleration() {
        return new Acceleration(mAccelerationBiasZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets estimated accelerometer bias resolved around z axis.
     *
     * @param accelerationBiasZ estimated accelerometer bias resolved
     *                          around z axis.
     */
    public void setAccelerationBiasZ(final Acceleration accelerationBiasZ) {
        mAccelerationBiasZ = AccelerationConverter.convert(
                accelerationBiasZ.getValue().doubleValue(),
                accelerationBiasZ.getUnit(),
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Sets estimated accelerometer bias coordinates.
     *
     * @param accelerationBiasX estimated accelerometer bias resolved around x axis.
     * @param accelerationBiasY estimated accelerometer bias resolved around y axis.
     * @param accelerationBiasZ estimated accelerometer bias resolved around z axis.
     */
    public void setAccelerationBiasCoordinates(
            final Acceleration accelerationBiasX, final Acceleration accelerationBiasY,
            final Acceleration accelerationBiasZ) {
        setAccelerationBiasX(accelerationBiasX);
        setAccelerationBiasY(accelerationBiasY);
        setAccelerationBiasZ(accelerationBiasZ);
    }

    /**
     * Gets estimated gyroscope bias resolved around x axis.
     *
     * @param result instance where estimated gyroscope bias resolved around x axis will
     *               be stored.
     */
    public void getAngularSpeedGyroBiasX(final AngularSpeed result) {
        result.setValue(mGyroBiasX);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated gyroscope bias resolved around x axis.
     *
     * @return estimated gyroscope bias resolved around x axis.
     */
    public AngularSpeed getAngularSpeedGyroBiasX() {
        return new AngularSpeed(mGyroBiasX, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets estimated gyroscope bias resolved around x axis.
     *
     * @param gyroBiasX estimated gyroscope bias resolved around x axis.
     */
    public void setGyroBiasX(final AngularSpeed gyroBiasX) {
        mGyroBiasX = AngularSpeedConverter.convert(
                gyroBiasX.getValue().doubleValue(),
                gyroBiasX.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated gyroscope bias resolved around y axis.
     *
     * @param result instance where estimated gyroscope bias resolved around y axis will
     *               be stored.
     */
    public void getAngularSpeedGyroBiasY(final AngularSpeed result) {
        result.setValue(mGyroBiasY);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated gyroscope bias resolved around y axis.
     *
     * @return estimated gyroscope bias resolved around y axis.
     */
    public AngularSpeed getAngularSpeedGyroBiasY() {
        return new AngularSpeed(mGyroBiasY, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets estimated gyroscope bias resolved around y axis.
     *
     * @param gyroBiasY estimated gyroscope bias resolved around y axis.
     */
    public void setGyroBiasY(final AngularSpeed gyroBiasY) {
        mGyroBiasY = AngularSpeedConverter.convert(
                gyroBiasY.getValue().doubleValue(),
                gyroBiasY.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated gyroscope bias resolved around z axis.
     *
     * @param result instance where estimated gyroscope bias resolved around z axis will
     *               be stored.
     */
    public void getAngularSpeedGyroBiasZ(final AngularSpeed result) {
        result.setValue(mGyroBiasZ);
        result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Gets estimated gyroscope bias resolved around z axis.
     *
     * @return estimated gyroscope bias resolved around z axis.
     */
    public AngularSpeed getAngularSpeedGyroBiasZ() {
        return new AngularSpeed(mGyroBiasZ, AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets estimated gyroscope bias resolved around z axis.
     *
     * @param gyroBiasZ estimated gyroscope bias resolved around z axis.
     */
    public void setGyroBiasZ(final AngularSpeed gyroBiasZ) {
        mGyroBiasZ = AngularSpeedConverter.convert(
                gyroBiasZ.getValue().doubleValue(),
                gyroBiasZ.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
    }

    /**
     * Sets estimated gyroscope bias coordinates.
     *
     * @param gyroBiasX estimated gyroscope bias resolved around x axis.
     * @param gyroBiasY estimated gyroscope bias resolved around y axis.
     * @param gyroBiasZ estimated gyroscope bias resolved around z axis.
     */
    public void setGyroBiasCoordinates(
            final AngularSpeed gyroBiasX, final AngularSpeed gyroBiasY,
            final AngularSpeed gyroBiasZ) {
        setGyroBiasX(gyroBiasX);
        setGyroBiasY(gyroBiasY);
        setGyroBiasZ(gyroBiasZ);
    }

    /**
     * Gets estimated receiver clock offset.
     *
     * @param result instance where estimated receiver clock offset will be stored.
     */
    public void getReceiverClockOffsetAsDistance(final Distance result) {
        result.setValue(mReceiverClockOffset);
        result.setUnit(DistanceUnit.METER);
    }

    /**
     * Gets estimated receiver clock offset.
     *
     * @return estimated receiver clock offset.
     */
    public Distance getReceiverClockOffsetAsDistance() {
        return new Distance(mReceiverClockOffset, DistanceUnit.METER);
    }

    /**
     * Sets estimated receiver clock offset.
     *
     * @param receiverClockOffset estimated receiver clock offset.
     */
    public void setReceiverClockOffset(final Distance receiverClockOffset) {
        mReceiverClockOffset = DistanceConverter.convert(
                receiverClockOffset.getValue().doubleValue(),
                receiverClockOffset.getUnit(), DistanceUnit.METER);
    }

    /**
     * Gets estimated receiver clock drift.
     *
     * @param result instance where estimated receiver clock drift will be stored.
     */
    public void getReceiverClockDriftAsSpeed(final Speed result) {
        result.setValue(mReceiverClockDrift);
        result.setUnit(SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets estimated receiver clock drift.
     *
     * @return estimated receiver clock drift.
     */
    public Speed getReceiverClockDriftAsSpeed() {
        return new Speed(mReceiverClockDrift, SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Sets estimated receiver clock drift.
     *
     * @param receiverClockDrift estimated receiver clock drift.
     */
    public void setReceiverClockDrift(final Speed receiverClockDrift) {
        mReceiverClockDrift = SpeedConverter.convert(
                receiverClockDrift.getValue().doubleValue(),
                receiverClockDrift.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Gets GNSS estimation from data contained into this instance.
     *
     * @param result instance where GNSS estimation data will be stored.
     */
    public void getGNSSEstimation(final GNSSEstimation result) {
        result.setPositionCoordinates(mX, mY, mZ);
        result.setVelocityCoordinates(mVx, mVy, mVz);
        result.setClockOffset(mReceiverClockOffset);
        result.setClockDrift(mReceiverClockDrift);
    }

    /**
     * Gets GNSS estimation from data contained into this instance.
     *
     * @return a new GNSS estimation instance.
     */
    public GNSSEstimation getGNSSEstimation() {
        return new GNSSEstimation(mX, mY, mZ, mVx, mVy, mVz, mReceiverClockOffset,
                mReceiverClockDrift);
    }

    /**
     * Sets GNSS estimation data into this instance.
     *
     * @param gnssEstimation GNSS estimation data to be set.
     */
    public void setGNSSEstimation(final GNSSEstimation gnssEstimation) {
        mX = gnssEstimation.getX();
        mY = gnssEstimation.getY();
        mZ = gnssEstimation.getZ();

        mVx = gnssEstimation.getVx();
        mVy = gnssEstimation.getVy();
        mVz = gnssEstimation.getVz();

        mReceiverClockOffset = gnssEstimation.getClockOffset();
        mReceiverClockDrift = gnssEstimation.getClockDrift();
    }

    /**
     * Copies this instance data into provided instance.
     *
     * @param output destination instance where data will be copied to.
     */
    public void copyTo(final INSTightlyCoupledKalmanState output) {
        output.copyFrom(this);
    }

    /**
     * Copies data of provided instance into this instance.
     *
     * @param input instance to copy data from.
     */
    public void copyFrom(final INSTightlyCoupledKalmanState input) {
        // copy coordinate transformation matrix
        if (input.mBodyToEcefCoordinateTransformationMatrix == null) {
            mBodyToEcefCoordinateTransformationMatrix = null;
        } else {
            if (mBodyToEcefCoordinateTransformationMatrix == null) {
                mBodyToEcefCoordinateTransformationMatrix =
                        new Matrix(input.mBodyToEcefCoordinateTransformationMatrix);
            } else {
                mBodyToEcefCoordinateTransformationMatrix.copyFrom(
                        input.mBodyToEcefCoordinateTransformationMatrix);
            }
        }

        mVx = input.mVx;
        mVy = input.mVy;
        mVz = input.mVz;

        mX = input.mX;
        mY = input.mY;
        mZ = input.mZ;

        mAccelerationBiasX = input.mAccelerationBiasX;
        mAccelerationBiasY = input.mAccelerationBiasY;
        mAccelerationBiasZ = input.mAccelerationBiasZ;

        mGyroBiasX = input.mGyroBiasX;
        mGyroBiasY = input.mGyroBiasY;
        mGyroBiasZ = input.mGyroBiasZ;

        mReceiverClockOffset = input.mReceiverClockOffset;
        mReceiverClockDrift = input.mReceiverClockDrift;

        // copy covariance
        if (input.mCovariance == null) {
            mCovariance = null;
        } else {
            if (mCovariance == null) {
                mCovariance = new Matrix(input.mCovariance);
            } else {
                mCovariance.copyFrom(input.mCovariance);
            }
        }
    }

    /**
     * Computes and returns hash code for this instance. Hash codes are almost unique
     * values that are useful for fast classification and storage of objects in collections.
     *
     * @return Hash code.
     */
    @Override
    public int hashCode() {
        return Objects.hash(mBodyToEcefCoordinateTransformationMatrix,
                mVx, mVy, mVz, mX, mY, mZ,
                mAccelerationBiasX, mAccelerationBiasY, mAccelerationBiasZ,
                mGyroBiasX, mGyroBiasY, mGyroBiasZ,
                mReceiverClockOffset, mReceiverClockDrift, mCovariance);
    }

    /**
     * Checks if provided object is a INSTightlyCoupledKalmanState having exactly the same
     * contents as this instance.
     *
     * @param obj Object to be compared.
     * @return true if both objects are considered to be equal, false otherwise.
     */
    @Override
    public boolean equals(Object obj) {
        if (this == obj) {
            return true;
        }
        if (obj == null || getClass() != obj.getClass()) {
            return false;
        }
        INSTightlyCoupledKalmanState other = (INSTightlyCoupledKalmanState) obj;
        return equals(other);
    }

    /**
     * Checks if provided instance has exactly the same contents as this instance.
     *
     * @param other instance to be compared.
     * @return true if both instances are considered to be equal, false otherwise.
     */
    public boolean equals(final INSTightlyCoupledKalmanState other) {
        return equals(other, 0.0);
    }

    /**
     * Checks if provided instance has contents similar to this instance up to provided
     * threshold value.
     *
     * @param other     instance to be compared.
     * @param threshold maximum difference allowed for values.
     * @return true if both instances are considered to be equal (up to provided threshold),
     * false otherwise.
     */
    public boolean equals(final INSTightlyCoupledKalmanState other,
                          final double threshold) {
        if (other == null) {
            return false;
        }

        return Math.abs(mVx - other.mVx) <= threshold
                && Math.abs(mVy - other.mVy) <= threshold
                && Math.abs(mVz - other.mVz) <= threshold
                && Math.abs(mX - other.mX) <= threshold
                && Math.abs(mY - other.mY) <= threshold
                && Math.abs(mZ - other.mZ) <= threshold
                && Math.abs(mAccelerationBiasX - other.mAccelerationBiasX) <= threshold
                && Math.abs(mAccelerationBiasY - other.mAccelerationBiasY) <= threshold
                && Math.abs(mAccelerationBiasZ - other.mAccelerationBiasZ) <= threshold
                && Math.abs(mGyroBiasX - other.mGyroBiasX) <= threshold
                && Math.abs(mGyroBiasY - other.mGyroBiasY) <= threshold
                && Math.abs(mGyroBiasZ - other.mGyroBiasZ) <= threshold
                && Math.abs(mReceiverClockOffset - other.mReceiverClockOffset) <= threshold
                && Math.abs(mReceiverClockDrift - other.mReceiverClockDrift) <= threshold
                && other.mBodyToEcefCoordinateTransformationMatrix != null &&
                other.mBodyToEcefCoordinateTransformationMatrix.equals(mBodyToEcefCoordinateTransformationMatrix,
                        threshold)
                && other.mCovariance != null && other.mCovariance.equals(mCovariance, threshold);
    }

    /**
     * Makes a copy of this instance.
     *
     * @return a copy of this instance.
     * @throws CloneNotSupportedException if clone fails for some reason.
     */
    @Override
    protected Object clone() throws CloneNotSupportedException {
        final INSTightlyCoupledKalmanState result =
                (INSTightlyCoupledKalmanState) super.clone();
        copyTo(result);
        return result;
    }
}
