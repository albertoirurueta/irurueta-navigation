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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.gnss.ECEFPositionAndVelocity;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleConverter;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

/**
 * Implements one cycle of the loosely coupled INS/GNSS
 * Kalman filter plus closed-loop correction of all inertial states.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/MATLAB-Codes/blob/master/LC_KF_Epoch.m
 */
public class INSLooselyCoupledKalmanEpochEstimator {

    /**
     * Speed of light in the vacuum expressed in meters per second (m/s).
     */
    public static final double SPEED_OF_LIGHT = Constants.SPEED_OF_LIGHT;

    /**
     * Earth rotation rate expressed in radians per second (rad/s).
     */
    public static final double EARTH_ROTATION_RATE = Constants.EARTH_ROTATION_RATE;

    /**
     * The equatorial radius of WGS84 ellipsoid (6378137 m) defining Earth's shape.
     */
    public static final double EARTH_EQUATORIAL_RADIUS_WGS84 = Constants.EARTH_EQUATORIAL_RADIUS_WGS84;

    /**
     * Earth eccentricity as defined on the WGS84 ellipsoid.
     */
    public static final double EARTH_ECCENTRICITY = Constants.EARTH_ECCENTRICITY;

    /**
     * Number of components of position + velocity.
     */
    private static final int POS_AND_VEL_COMPONENTS = 6;

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        final INSLooselyCoupledKalmanState result = new INSLooselyCoupledKalmanState();
        estimate(userPosition, userVelocity, propagationInterval, previousState,
                bodyKinematics, config, result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {

        final double fx = bodyKinematics.getFx();
        final double fy = bodyKinematics.getFy();
        final double fz = bodyKinematics.getFz();

        estimate(userPosition, userVelocity, propagationInterval, previousState,
                fx, fy, fz, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        final INSLooselyCoupledKalmanState result = new INSLooselyCoupledKalmanState();
        estimate(userPosition, userVelocity, propagationInterval, previousState,
                bodyKinematics, previousLatitude, config, result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {

        final double fx = bodyKinematics.getFx();
        final double fy = bodyKinematics.getFy();
        final double fz = bodyKinematics.getFz();

        estimate(userPosition, userVelocity, propagationInterval, previousState,
                fx, fy, fz, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        final INSLooselyCoupledKalmanState result = new INSLooselyCoupledKalmanState();
        estimate(userPosition, userVelocity, propagationInterval,
                previousState, fx, fy, fz, config, result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {

        final double x = userPosition.getX();
        final double y = userPosition.getY();
        final double z = userPosition.getZ();

        final double vx = userVelocity.getVx();
        final double vy = userVelocity.getVy();
        final double vz = userVelocity.getVz();

        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                fx, fy, fz, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        final INSLooselyCoupledKalmanState result = new INSLooselyCoupledKalmanState();
        estimate(userPosition, userVelocity, propagationInterval, previousState,
                fx, fy, fz, previousLatitude, config, result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        GNSS estimated ECEF user position.
     * @param userVelocity        GNSS estimated ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {

        final double x = userPosition.getX();
        final double y = userPosition.getY();
        final double z = userPosition.getZ();

        final double vx = userVelocity.getVx();
        final double vy = userVelocity.getVy();
        final double vz = userVelocity.getVz();

        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                fx, fy, fz, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        final INSLooselyCoupledKalmanState result = new INSLooselyCoupledKalmanState();
        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                bodyKinematics, config, result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {

        final double fx = bodyKinematics.getFx();
        final double fy = bodyKinematics.getFy();
        final double fz = bodyKinematics.getFz();

        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                fx, fy, fz, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        final INSLooselyCoupledKalmanState result = new INSLooselyCoupledKalmanState();
        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                bodyKinematics, previousLatitude, config, result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {

        final double fx = bodyKinematics.getFx();
        final double fy = bodyKinematics.getFy();
        final double fz = bodyKinematics.getFz();

        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                fx, fy, fz, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        final INSLooselyCoupledKalmanState result = new INSLooselyCoupledKalmanState();
        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                fx, fy, fz, config, result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {

        final NEDPosition prevNedPosition = new NEDPosition();
        final NEDVelocity prevNedVelocity = new NEDVelocity();
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(
                previousState.getX(), previousState.getY(), previousState.getZ(),
                previousState.getVx(), previousState.getVy(), previousState.getVz(),
                prevNedPosition, prevNedVelocity);

        final double previousLatitude = prevNedPosition.getLatitude();

        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                fx, fy, fz, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        final INSLooselyCoupledKalmanState result = new INSLooselyCoupledKalmanState();
        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                fx, fy, fz, previousLatitude, config, result);
        return result;
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {

        final Matrix omegaIe = Utils.skewMatrix(
                new double[]{0.0, 0.0, EARTH_ROTATION_RATE});

        // SYSTEM PROPAGATION PHASE

        // 1. Determine transition matrix using (14.50) (first-order approx)
        final Matrix phiMatrix = Matrix.identity(
                INSLooselyCoupledKalmanState.NUM_PARAMS,
                INSLooselyCoupledKalmanState.NUM_PARAMS);

        final Matrix tmp1 = omegaIe.multiplyByScalarAndReturnNew(
                propagationInterval);
        final Matrix tmp2 = phiMatrix.getSubmatrix(0, 0,
                2, 2);
        tmp2.subtract(tmp1);

        phiMatrix.setSubmatrix(0, 0,
                2, 2, tmp2);

        final Matrix estCbeOld = previousState
                .getBodyToEcefCoordinateTransformationMatrix();
        tmp1.copyFrom(estCbeOld);
        tmp1.multiplyByScalar(propagationInterval);

        phiMatrix.setSubmatrix(0, 12,
                2, 14, tmp1);
        phiMatrix.setSubmatrix(3, 9,
                5, 11, tmp1);

        final Matrix measFibb = new Matrix(BodyKinematics.COMPONENTS, 1);
        measFibb.setElementAtIndex(0, fx);
        measFibb.setElementAtIndex(1, fy);
        measFibb.setElementAtIndex(2, fz);

        estCbeOld.multiply(measFibb, tmp1);

        Utils.skewMatrix(tmp1, tmp2);
        tmp2.multiplyByScalar(-propagationInterval);

        phiMatrix.setSubmatrix(3, 0,
                5, 2, tmp2);

        phiMatrix.getSubmatrix(3, 3,
                5, 5, tmp1);
        tmp2.copyFrom(omegaIe);
        tmp2.multiplyByScalar(2.0 * propagationInterval);
        tmp1.subtract(tmp2);
        phiMatrix.setSubmatrix(3, 3,
                5, 5, tmp1);

        final double sinPrevLat = Math.sin(previousLatitude);
        final double cosPrevLat = Math.cos(previousLatitude);
        final double sinPrevLat2 = sinPrevLat * sinPrevLat;
        final double cosPrevLat2 = cosPrevLat * cosPrevLat;

        // From (2.137)
        final double geocentricRadius = EARTH_EQUATORIAL_RADIUS_WGS84
                / Math.sqrt(1.0 - Math.pow(EARTH_ECCENTRICITY * sinPrevLat, 2.0))
                * Math.sqrt(cosPrevLat2
                + Math.pow(1.0 - EARTH_ECCENTRICITY * EARTH_ECCENTRICITY, 2.0) * sinPrevLat2);

        final double prevX = previousState.getX();
        final double prevY = previousState.getY();
        final double prevZ = previousState.getZ();
        final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(
                prevX, prevY, prevZ);

        final double previousPositionNorm = Math.sqrt(prevX * prevX +
                prevY * prevY + prevZ * prevZ);

        final Matrix estRebeOld = new Matrix(ECEFPosition.COMPONENTS, 1);
        estRebeOld.setElementAtIndex(0, prevX);
        estRebeOld.setElementAtIndex(1, prevY);
        estRebeOld.setElementAtIndex(2, prevZ);

        final Matrix g = gravity.asMatrix();
        g.multiplyByScalar(-2.0 * propagationInterval / geocentricRadius);

        final Matrix estRebeOldTrans = estRebeOld.transposeAndReturnNew();
        estRebeOldTrans.multiplyByScalar(1.0 / previousPositionNorm);

        g.multiply(estRebeOldTrans, tmp1);

        phiMatrix.setSubmatrix(3, 6,
                5, 8, tmp1);

        for (int i = 0; i < ECEFPosition.COMPONENTS; i++) {
            phiMatrix.setElementAt(6 + i, 3 + i, propagationInterval);
        }


        // 2. Determine approximate system noise covariance matrix using (14.82)
        final Matrix qPrimeMatrix = new Matrix(
                INSLooselyCoupledKalmanState.NUM_PARAMS,
                INSLooselyCoupledKalmanState.NUM_PARAMS);

        final double gyroNoisePSD = config.getGyroNoisePSD();
        final double gyroNoiseValue = gyroNoisePSD * propagationInterval;
        for (int i = 0; i < 3; i++) {
            qPrimeMatrix.setElementAt(i, i, gyroNoiseValue);
        }

        final double accelNoisePSD = config.getAccelerometerNoisePSD();
        final double accelNoiseValue = accelNoisePSD * propagationInterval;
        for (int i = 3; i < 6; i++) {
            qPrimeMatrix.setElementAt(i, i, accelNoiseValue);
        }

        final double accelBiasPSD = config.getAccelerometerBiasPSD();
        final double accelBiasValue = accelBiasPSD * propagationInterval;
        for (int i = 9; i < 12; i++) {
            qPrimeMatrix.setElementAt(i, i, accelBiasValue);
        }

        final double gyroBiasPSD = config.getGyroBiasPSD();
        final double gyroBiasValue = gyroBiasPSD * propagationInterval;
        for (int i = 12; i < 15; i++) {
            qPrimeMatrix.setElementAt(i, i, gyroBiasValue);
        }


        // 3. Propagate state estimates using (3.14) noting that all states are zero
        // due to closed-loop correction.
        // x_est_propagated(1:15, 1) = 0;

        // 4. Propagate state estimation error covariance matrix using (3.46)
        final Matrix pMatrixOld = previousState.getCovariance();

        qPrimeMatrix.multiplyByScalar(0.5);

        final Matrix tmp3 = pMatrixOld.addAndReturnNew(qPrimeMatrix);
        final Matrix pMatrixPropagated = phiMatrix.multiplyAndReturnNew(tmp3);

        phiMatrix.transpose();
        pMatrixPropagated.multiply(phiMatrix);

        pMatrixPropagated.add(qPrimeMatrix);


        // MEASUREMENT UPDATE PHASE

        // 5. Set-up measurement matrix using (14.115)
        final Matrix h = new Matrix(POS_AND_VEL_COMPONENTS,
                INSLooselyCoupledKalmanState.NUM_PARAMS);
        for (int i = 0; i < 3; i++) {
            h.setElementAt(i, POS_AND_VEL_COMPONENTS + i, -1.0);
            int j = ECEFPosition.COMPONENTS + i;
            h.setElementAt(j, j, -1.0);
        }

        // 6. Set-up measurement noise covariance matrix assuming all components of
        // GNSS position and velocity are independent and have equal variance.
        final Matrix r = new Matrix(POS_AND_VEL_COMPONENTS, POS_AND_VEL_COMPONENTS);

        final double posMeasSD = config.getPositionNoiseSD();
        final double posMeasSD2 = posMeasSD * posMeasSD;
        for (int i = 0; i < ECEFPosition.COMPONENTS; i++) {
            r.setElementAt(i, i, posMeasSD2);
        }

        final double velMeasSD = config.getVelocityNoiseSD();
        final double velMeasSD2 = velMeasSD * velMeasSD;
        for (int i = ECEFPosition.COMPONENTS; i < POS_AND_VEL_COMPONENTS; i++) {
            r.setElementAt(i, i, velMeasSD2);
        }

        // 7. Calculate Kalman gain using (3.21)
        final Matrix hTrans = h.transposeAndReturnNew();

        final Matrix tmp4 = h.multiplyAndReturnNew(pMatrixPropagated);
        tmp4.multiply(hTrans);
        tmp4.add(r);

        final Matrix tmp5 = Utils.inverse(tmp4);

        final Matrix k = pMatrixPropagated.multiplyAndReturnNew(hTrans);
        k.multiply(tmp5);

        // 8. Formulate measurement innovations using (14.102), noting that zero
        // lever arm is assumed here
        final double prevVx = previousState.getVx();
        final double prevVy = previousState.getVy();
        final double prevVz = previousState.getVz();

        final Matrix deltaZ = new Matrix(POS_AND_VEL_COMPONENTS, 1);
        deltaZ.setElementAtIndex(0, x - prevX);
        deltaZ.setElementAtIndex(1, y - prevY);
        deltaZ.setElementAtIndex(2, z - prevZ);
        deltaZ.setElementAtIndex(3, vx - prevVx);
        deltaZ.setElementAtIndex(4, vy - prevVy);
        deltaZ.setElementAtIndex(5, vz - prevVz);

        // 9. Update state estimates using (3.24)
        // x_est_new = x_est_propagated + K_matrix * delta_z;
        final Matrix xEstNew = k.multiplyAndReturnNew(deltaZ);

        // 10. Update state estimation error covariance matrix using (3.25)
        k.multiply(h);
        final Matrix pNew = Matrix.identity(INSLooselyCoupledKalmanState.NUM_PARAMS,
                INSLooselyCoupledKalmanState.NUM_PARAMS);
        pNew.subtract(k);
        pNew.multiply(pMatrixPropagated);

        // CLOSED-LOOP CORRECTION

        // Correct attitude, velocity, and position using (14.7-9)
        final Matrix tmp6 = xEstNew.getSubmatrix(0, 0,
                2, 0);
        Matrix tmp7 = Utils.skewMatrix(tmp6);

        final Matrix estCbeNew = Matrix.identity(ECEFPosition.COMPONENTS,
                ECEFPosition.COMPONENTS);
        estCbeNew.subtract(tmp7);
        estCbeNew.multiply(estCbeOld);

        final double newVx = prevVx - xEstNew.getElementAtIndex(3);
        final double newVy = prevVy - xEstNew.getElementAtIndex(4);
        final double newVz = prevVz - xEstNew.getElementAtIndex(5);

        final double newX = prevX - xEstNew.getElementAtIndex(6);
        final double newY = prevY - xEstNew.getElementAtIndex(7);
        final double newZ = prevZ - xEstNew.getElementAtIndex(8);

        // Update IMU bias estimates
        final double newAccelerationBiasX = previousState.getAccelerationBiasX()
                + xEstNew.getElementAtIndex(9);
        final double newAccelerationBiasY = previousState.getAccelerationBiasY()
                + xEstNew.getElementAtIndex(10);
        final double newAccelerationBiasZ = previousState.getAccelerationBiasZ()
                + xEstNew.getElementAtIndex(11);

        final double newGyroBiasX = previousState.getGyroBiasX()
                + xEstNew.getElementAtIndex(12);
        final double newGyroBiasY = previousState.getGyroBiasY()
                + xEstNew.getElementAtIndex(13);
        final double newGyroBiasZ = previousState.getGyroBiasZ()
                + xEstNew.getElementAtIndex(14);


        // set result values
        result.setBodyToEcefCoordinateTransformationMatrix(estCbeNew);
        result.setVelocityCoordinates(newVx, newVy, newVz);
        result.setPositionCoordinates(newX, newY, newZ);
        result.setAccelerationBiasCoordinates(newAccelerationBiasX,
                newAccelerationBiasY, newAccelerationBiasZ);
        result.setGyroBiasCoordinates(newGyroBiasX, newGyroBiasY,
                newGyroBiasZ);
        result.setCovariance(pNew);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, convertTime(propagationInterval),
                previousState, bodyKinematics, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition, userVelocity, convertTime(propagationInterval),
                previousState, bodyKinematics, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagaion interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, convertTime(propagationInterval),
                previousState, bodyKinematics, previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state of a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition, userVelocity, convertTime(propagationInterval),
                previousState, bodyKinematics, previousLatitude, config,
                result);
    }

    /**
     * Estimates the update of Kalman filter state of a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, convertTime(propagationInterval),
                previousState, fx, fy, fz, config);
    }

    /**
     * Estimates the update of Kalman filter state of a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition, userVelocity, convertTime(propagationInterval),
                previousState, fx, fy, fz, config, result);
    }

    /**
     * Estimates the update of Kalman filter state of a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, convertTime(propagationInterval),
                previousState, fx, fy, fz, previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state of a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition, userVelocity, convertTime(propagationInterval),
                previousState, fx, fy, fz, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state of a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(x, y, z, vx, vy, vz, convertTime(propagationInterval),
                previousState, bodyKinematics, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(x, y, z, vx, vy, vz, convertTime(propagationInterval), previousState,
                bodyKinematics, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(x, y, z, vx, vy, vz, convertTime(propagationInterval),
                previousState, bodyKinematics, previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(x, y, z, vx, vy, vz, convertTime(propagationInterval),
                previousState, bodyKinematics, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(x, y, z, vx, vy, vz, convertTime(propagationInterval),
                previousState, fx, fy, fz, config);
    }

    /**
     * Estimated the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(x, y, z, vx, vy, vz, convertTime(propagationInterval),
                previousState, fx, fy, fz, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(x, y, z, vx, vy, vz, convertTime(propagationInterval),
                previousState, fx, fy, fz, previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(x, y, z, vx, vy, vz, convertTime(propagationInterval),
                previousState, fx, fy, fz, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEf user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition.getInhomX(), userPosition.getInhomY(),
                userPosition.getInhomZ(), userVelocity.getVx(), userVelocity.getVy(),
                userVelocity.getVz(), propagationInterval, previousState, bodyKinematics,
                config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition.getInhomX(), userPosition.getInhomY(),
                userPosition.getInhomZ(), userVelocity.getVx(), userVelocity.getVy(),
                userVelocity.getVz(), propagationInterval, previousState,
                bodyKinematics, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition.getInhomX(), userPosition.getInhomY(),
                userPosition.getInhomZ(), userVelocity.getVx(), userVelocity.getVy(),
                userVelocity.getVz(), propagationInterval,
                previousState, bodyKinematics, previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition.getInhomX(), userPosition.getInhomY(),
                userPosition.getInhomZ(), userVelocity.getVx(), userVelocity.getVy(),
                userVelocity.getVz(), propagationInterval, previousState,
                bodyKinematics, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition.getInhomX(), userPosition.getInhomY(),
                userPosition.getInhomZ(), userVelocity.getVx(), userVelocity.getVy(),
                userVelocity.getVz(), propagationInterval, previousState,
                fx, fy, fz, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition.getInhomX(), userPosition.getInhomY(),
                userPosition.getInhomZ(), userVelocity.getVx(), userVelocity.getVy(),
                userVelocity.getVz(), propagationInterval, previousState,
                fx, fy, fz, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition.getInhomX(), userPosition.getInhomY(),
                userPosition.getInhomZ(), userVelocity.getVx(), userVelocity.getVy(),
                userVelocity.getVz(), propagationInterval, previousState,
                fx, fy, fz, previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition.getInhomX(), userPosition.getInhomY(),
                userPosition.getInhomZ(), userVelocity.getVx(), userVelocity.getVy(),
                userVelocity.getVz(), propagationInterval, previousState,
                fx, fy, fz, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition.getInhomX(), userPosition.getInhomY(),
                userPosition.getInhomZ(), userVelocity.getVx(), userVelocity.getVy(),
                userVelocity.getVz(), propagationInterval, previousState,
                bodyKinematics, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition.getInhomX(), userPosition.getInhomY(),
                userPosition.getInhomZ(), userVelocity.getVx(),
                userVelocity.getVy(), userVelocity.getVz(),
                propagationInterval, previousState, bodyKinematics, config,
                result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition.getInhomX(), userPosition.getInhomY(),
                userPosition.getInhomZ(), userVelocity.getVx(), userVelocity.getVy(),
                userVelocity.getVz(), propagationInterval, previousState,
                bodyKinematics, previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition.getInhomX(), userPosition.getInhomY(),
                userPosition.getInhomZ(), userVelocity.getVx(), userVelocity.getVy(),
                userVelocity.getVz(), propagationInterval, previousState,
                bodyKinematics, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition.getInhomX(), userPosition.getInhomY(),
                userPosition.getInhomZ(), userVelocity.getVx(), userVelocity.getVy(),
                userVelocity.getVz(), propagationInterval, previousState,
                fx, fy, fz, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are anumerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition.getInhomX(), userPosition.getInhomY(),
                userPosition.getInhomZ(), userVelocity.getVx(), userVelocity.getVy(),
                userVelocity.getVz(), propagationInterval, previousState,
                fx, fy, fz, config, result);
    }

    /**
     * Estimtes the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition.getInhomX(), userPosition.getInhomY(),
                userPosition.getInhomZ(), userVelocity.getVx(), userVelocity.getVy(),
                userVelocity.getVz(), propagationInterval, previousState,
                fx, fy, fz, previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition.getInhomX(), userPosition.getInhomY(),
                userPosition.getInhomZ(), userVelocity.getVx(), userVelocity.getVy(),
                userVelocity.getVz(), propagationInterval, previousState,
                fx, fy, fz, previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity.getX(), positionAndVelocity.getY(),
                positionAndVelocity.getZ(), positionAndVelocity.getVx(),
                positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, bodyKinematics,
                config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(positionAndVelocity.getX(), positionAndVelocity.getY(),
                positionAndVelocity.getZ(), positionAndVelocity.getVx(),
                positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, bodyKinematics,
                config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity.getX(), positionAndVelocity.getY(),
                positionAndVelocity.getZ(), positionAndVelocity.getVx(),
                positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, bodyKinematics,
                previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(positionAndVelocity.getX(), positionAndVelocity.getY(),
                positionAndVelocity.getZ(), positionAndVelocity.getVx(),
                positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, bodyKinematics,
                previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity.getX(), positionAndVelocity.getY(),
                positionAndVelocity.getZ(), positionAndVelocity.getVx(),
                positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, fx, fy, fz, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(positionAndVelocity.getX(), positionAndVelocity.getY(),
                positionAndVelocity.getZ(), positionAndVelocity.getVx(),
                positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, fx, fy, fz, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity.getX(), positionAndVelocity.getY(),
                positionAndVelocity.getZ(), positionAndVelocity.getVx(),
                positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, fx, fy, fz, previousLatitude,
                config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(positionAndVelocity.getX(), positionAndVelocity.getY(),
                positionAndVelocity.getZ(), positionAndVelocity.getVx(),
                positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, fx, fy, fz, previousLatitude,
                config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity.getX(), positionAndVelocity.getY(),
                positionAndVelocity.getZ(), positionAndVelocity.getVx(),
                positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, bodyKinematics, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instace where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(positionAndVelocity.getX(), positionAndVelocity.getY(),
                positionAndVelocity.getZ(), positionAndVelocity.getVx(),
                positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, bodyKinematics,
                config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity.getX(), positionAndVelocity.getY(),
                positionAndVelocity.getZ(), positionAndVelocity.getVx(),
                positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, bodyKinematics,
                previousLatitude, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(positionAndVelocity.getX(), positionAndVelocity.getY(),
                positionAndVelocity.getZ(), positionAndVelocity.getVx(),
                positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, bodyKinematics,
                previousLatitude, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity.getX(), positionAndVelocity.getY(),
                positionAndVelocity.getZ(), positionAndVelocity.getVx(),
                positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, fx, fy, fz, config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(positionAndVelocity.getX(), positionAndVelocity.getY(),
                positionAndVelocity.getZ(), positionAndVelocity.getVx(),
                positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, fx, fy, fz, config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity.getX(), positionAndVelocity.getY(),
                positionAndVelocity.getZ(), positionAndVelocity.getVx(),
                positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, fx, fy, fz, previousLatitude,
                config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final double previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(positionAndVelocity.getX(), positionAndVelocity.getY(),
                positionAndVelocity.getZ(), positionAndVelocity.getVx(),
                positionAndVelocity.getVy(), positionAndVelocity.getVz(),
                propagationInterval, previousState, fx, fy, fz, previousLatitude,
                config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, propagationInterval,
                previousState, bodyKinematics, convertAngle(previousLatitude),
                config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition, userVelocity, propagationInterval, previousState,
                bodyKinematics, convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, propagationInterval,
                previousState, fx, fy, fz, convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        GNSS estimated ECEF user position.
     * @param userVelocity        GNSS estimated ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition, userVelocity, propagationInterval,
                previousState, fx, fy, fz, convertAngle(previousLatitude),
                config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                bodyKinematics, convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                bodyKinematics, convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                fx, fy, fz, convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                fx, fy, fz, convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagaion interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, propagationInterval,
                previousState, bodyKinematics, convertAngle(previousLatitude),
                config);
    }

    /**
     * Estimates the update of Kalman filter state of a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition, userVelocity, propagationInterval, previousState,
                bodyKinematics, convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state of a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, propagationInterval,
                previousState, fx, fy, fz, convertAngle(previousLatitude),
                config);
    }

    /**
     * Estimates the update of Kalman filter state of a single epoch.
     *
     * @param userPosition        ECEF user position.
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPosition userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition, userVelocity, propagationInterval, previousState,
                fx, fy, fz, convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                bodyKinematics, convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                bodyKinematics, convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                fx, fy, fz, convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param x                   ECEF x coordinate of user position expressed in
     *                            meters (m).
     * @param y                   ECEF y coordinate of user position expressed in
     *                            meters (m).
     * @param z                   ECEF z coordinate of user position expressed in
     *                            meters (m).
     * @param vx                  ECEF x coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vy                  ECEF y coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param vz                  ECEF z coordinate of user velocity expressed in
     *                            meters per second (m/s).
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final double x, final double y, final double z,
            final double vx, final double vy, final double vz,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(x, y, z, vx, vy, vz, propagationInterval, previousState,
                fx, fy, fz, convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, propagationInterval,
                previousState, bodyKinematics, convertAngle(previousLatitude),
                config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution expressed in radians (rad).
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition, userVelocity, propagationInterval, previousState,
                bodyKinematics, convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, propagationInterval,
                previousState, fx, fy, fz, convertAngle(previousLatitude),
                config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition, userVelocity, propagationInterval, previousState,
                fx, fy, fz, convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, propagationInterval,
                previousState, bodyKinematics, convertAngle(previousLatitude),
                config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition, userVelocity, propagationInterval, previousState,
                bodyKinematics, convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimtes the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(userPosition, userVelocity, propagationInterval,
                previousState, fx, fy, fz, convertAngle(previousLatitude),
                config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param userPosition        ECEF user position expressed in meters (m).
     * @param userVelocity        ECEF user velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final Point3D userPosition,
            final ECEFVelocity userVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(userPosition, userVelocity, propagationInterval, previousState,
                fx, fy, fz, convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity, propagationInterval, previousState,
                bodyKinematics, convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(positionAndVelocity, propagationInterval, previousState,
                bodyKinematics, convertAngle(previousLatitude), config,
                result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity, propagationInterval, previousState,
                fx, fy, fz, convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval expressed in seconds (s).
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final double propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(positionAndVelocity, propagationInterval, previousState,
                fx, fy, fz, convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity, propagationInterval, previousState,
                bodyKinematics, convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param bodyKinematics      body kinematics containing measured specific force
     *                            resolved along body frame axes.
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final BodyKinematics bodyKinematics,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(positionAndVelocity, propagationInterval, previousState,
                bodyKinematics, convertAngle(previousLatitude), config, result);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @return new state of Kalman filter.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static INSLooselyCoupledKalmanState estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config) throws AlgebraException {
        return estimate(positionAndVelocity, propagationInterval, previousState,
                fx, fy, fz, convertAngle(previousLatitude), config);
    }

    /**
     * Estimates the update of Kalman filter state for a single epoch.
     *
     * @param positionAndVelocity ECEF user position and velocity.
     * @param propagationInterval propagation interval.
     * @param previousState       previous Kalman filter state.
     * @param fx                  measured specific force resolved along body frame
     *                            x-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fy                  measured specific force resolved along body frame
     *                            y-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param fz                  measured specific force resolved along body frame
     *                            z-axis and expressed in meters per squared
     *                            second (m/s^2).
     * @param previousLatitude    previous latitude solution.
     * @param config              Loosely Coupled Kalman filter configuration.
     * @param result              instance where new state of Kalman filter will be
     *                            stored.
     * @throws AlgebraException if there are numerical instabilities.
     */
    public static void estimate(
            final ECEFPositionAndVelocity positionAndVelocity,
            final Time propagationInterval,
            final INSLooselyCoupledKalmanState previousState,
            final double fx, final double fy, final double fz,
            final Angle previousLatitude,
            final INSLooselyCoupledKalmanConfig config,
            final INSLooselyCoupledKalmanState result) throws AlgebraException {
        estimate(positionAndVelocity, propagationInterval, previousState,
                fx, fy, fz, convertAngle(previousLatitude), config, result);
    }

    /**
     * Converts time instance into a value expressed in seconds.
     *
     * @param time time instance to be converted.
     * @return time value expressed in seconds.
     */
    private static double convertTime(final Time time) {
        return TimeConverter.convert(time.getValue().doubleValue(),
                time.getUnit(), TimeUnit.SECOND);
    }

    /**
     * Converts angle instance into a value expressed in radians.
     *
     * @param angle angle instance to be converted.
     * @return angle value expressed in radians.
     */
    private static double convertAngle(final Angle angle) {
        return AngleConverter.convert(angle.getValue().doubleValue(),
                angle.getUnit(), AngleUnit.RADIANS);
    }
}
