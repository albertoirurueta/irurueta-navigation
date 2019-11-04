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
package com.irurueta.navigation.inertial.estimators;

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.ECEFGravity;
import com.irurueta.navigation.inertial.ECEFKinematics;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedConverter;
import com.irurueta.units.SpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

/**
 * Estimates body kinematics (specific force applied to a body and its angular rates) with respect and resolved
 * along ECEF-frame axes.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/InertialDemoECEF
 */
@SuppressWarnings("WeakerAccess")
public class ECEFKinematicsEstimator {

    /**
     * Earth rotation rate expressed in radians per second (rad/s).
     */
    public static final double EARTH_ROTATION_RATE = Constants.EARTH_ROTATION_RATE;

    /**
     * Scaling threshold.
     */
    private static final double SCALING_THRESHOLD = 2e-5;

    /**
     * Alpha threshold.
     */
    private static final double ALPHA_THRESHOLD = 1e-8;

    /**
     * Number of rows.
     */
    private static final int ROWS = 3;

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param x            cartesian x coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final double vx, final double vy, final double vz,
                         final double oldVx, final double oldVy, final double oldVz,
                         final double x, final double y, final double z,
                         final ECEFKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param c            boy-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param x            cartesian x coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final double vx, final double vy, final double vz,
                         final double oldVx, final double oldVy, final double oldVz,
                         final double x, final double y, final double z,
                         final ECEFKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public void estimate(final double timeInterval,
                         final ECEFFrame frame,
                         final CoordinateTransformation oldC,
                         final double oldVx, final double oldVy, final double oldVz,
                         final ECEFKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVx, oldVy, oldVz,
                result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public void estimate(final Time timeInterval,
                         final ECEFFrame frame,
                         final CoordinateTransformation oldC,
                         final double oldVx, final double oldVy, final double oldVz,
                         final ECEFKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVx, oldVy, oldVz,
                result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldFrame     body ECEF frame containing previous position, velocity and
     *                     body-to-ECEF frame coordinate transformation. Notice that
     *                     previous position contained in this frame is ignored.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final double timeInterval,
                         final ECEFFrame frame, final ECEFFrame oldFrame,
                         final ECEFKinematics result) {
        estimateKinematics(timeInterval, frame, oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldFrame     body ECEF frame containing previous position, velocity and
     *                     body-to-ECEF frame coordinate transformation. Notice that
     *                     previous position contained in this frame is ignored.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final Time timeInterval,
                         final ECEFFrame frame, final ECEFFrame oldFrame,
                         final ECEFKinematics result) {
        estimateKinematics(timeInterval, frame, oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final Speed vx, final Speed vy, final Speed vz,
                         final Speed oldVx, final Speed oldVy, final Speed oldVz,
                         final double x, final double y, final double z,
                         final ECEFKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final Speed vx, final Speed vy, final Speed vz,
                         final Speed oldVx, final Speed oldVy, final Speed oldVz,
                         final double x, final double y, final double z,
                         final ECEFKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz,
                oldVx, oldVy, oldVz, x, y, z, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public void estimate(final double timeInterval,
                         final ECEFFrame frame,
                         final CoordinateTransformation oldC,
                         final Speed oldVx, final Speed oldVy, final Speed oldVz,
                         final ECEFKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVx, oldVy, oldVz, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public void estimate(final Time timeInterval,
                         final ECEFFrame frame,
                         final CoordinateTransformation oldC,
                         final Speed oldVx, final Speed oldVy, final Speed oldVz,
                         final ECEFKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVx, oldVy, oldVz, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param position     body position expressed in meters (m) with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final double vx, final double vy, final double vz,
                         final double oldVx, final double oldVy, final double oldVz,
                         final Point3D position, final ECEFKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param position     body position expressed in meters (m) with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final double vx, final double vy, final double vz,
                         final double oldVx, final double oldVy, final double oldVz,
                         final Point3D position, final ECEFKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            boy-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @param position     body position expressed in meters (m) with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final Speed vx, final Speed vy, final Speed vz,
                         final Speed oldVx, final Speed oldVy, final Speed oldVz,
                         final Point3D position, final ECEFKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param c            boy-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @param position     body position expressed in meters (m) with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final Speed vx, final Speed vy, final Speed vz,
                         final Speed oldVx, final Speed oldVy, final Speed oldVz,
                         final Point3D position, final ECEFKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param x            cartesian x coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public ECEFKinematics estimateAndReturnNew(final double timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final double vx, final double vy, final double vz,
                                               final double oldVx, final double oldVy, final double oldVz,
                                               final double x, final double y, final double z) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between.
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param x            cartesian x coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public ECEFKinematics estimateAndReturnNew(final Time timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final double vx, final double vy, final double vz,
                                               final double oldVx, final double oldVy, final double oldVz,
                                               final double x, final double y, final double z) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz,
                oldVx, oldVy, oldVz, x, y, z);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public ECEFKinematics estimateAndReturnNew(final double timeInterval,
                                               final ECEFFrame frame,
                                               final CoordinateTransformation oldC,
                                               final double oldVx, final double oldVy,
                                               final double oldVz) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldC, oldVx, oldVy,
                oldVz);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public ECEFKinematics estimateAndReturnNew(final Time timeInterval,
                                               final ECEFFrame frame,
                                               final CoordinateTransformation oldC,
                                               final double oldVx, final double oldVy,
                                               final double oldVz) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldC,
                oldVx, oldVy, oldVz);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldFrame     body ECEF frame containing previous position, velocity and
     *                     body-to-ECEF frame coordinate transformation. Notice that
     *                     previous position contained in this frame is ignored.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public ECEFKinematics estimateAndReturnNew(final double timeInterval,
                                               final ECEFFrame frame,
                                               final ECEFFrame oldFrame) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldFrame);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldFrame     body ECEF frame containing previous position, velocity and
     *                     body-to-ECEF frame coordinate transformation. Notice that
     *                     previous position contained in this frame is ignored.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public ECEFKinematics estimateAndReturnNew(final Time timeInterval,
                                               final ECEFFrame frame,
                                               final ECEFFrame oldFrame) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldFrame);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public ECEFKinematics estimateAndReturnNew(final double timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final Speed vx, final Speed vy, final Speed vz,
                                               final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                               final double x, final double y, final double z) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz,
                oldVx, oldVy, oldVz, x, y, z);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public ECEFKinematics estimateAndReturnNew(final Time timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final Speed vx, final Speed vy, final Speed vz,
                                               final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                               final double x, final double y, final double z) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz,
                oldVx, oldVy, oldVz, x, y, z);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public ECEFKinematics estimateAndReturnNew(final double timeInterval,
                                               final ECEFFrame frame,
                                               final CoordinateTransformation oldC,
                                               final Speed oldVx, final Speed oldVy, final Speed oldVz) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldC, oldVx, oldVy, oldVz);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public ECEFKinematics estimateAndReturnNew(final Time timeInterval,
                                               final ECEFFrame frame,
                                               final CoordinateTransformation oldC,
                                               final Speed oldVx, final Speed oldVy, final Speed oldVz) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldC, oldVx, oldVy, oldVz);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param position     body position expressed in meters (m) with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public ECEFKinematics estimateAndReturnNew(final double timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final double vx, final double vy, final double vz,
                                               final double oldVx, final double oldVy, final double oldVz,
                                               final Point3D position) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz,
                oldVx, oldVy, oldVz, position);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param position     body position expressed in meters (m) with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public ECEFKinematics estimateAndReturnNew(final Time timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final double vx, final double vy, final double vz,
                                               final double oldVx, final double oldVy, final double oldVz,
                                               final Point3D position) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            boy-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @param position     body position expressed in meters (m) with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public ECEFKinematics estimateAndReturnNew(final double timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final Speed vx, final Speed vy, final Speed vz,
                                               final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                               final Point3D position) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz,
                oldVx, oldVy, oldVz, position);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param c            boy-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @param position     body position expressed in meters (m) with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public ECEFKinematics estimateAndReturnNew(final Time timeInterval,
                                               final CoordinateTransformation c,
                                               final CoordinateTransformation oldC,
                                               final Speed vx, final Speed vy, final Speed vz,
                                               final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                               final Point3D position) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz,
                oldVx, oldVy, oldVz, position);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECEF-frame coordinate transformation.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECEF-frame axes.
     * @param vy           y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECEF-frame axes.
     * @param vz           z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECEF-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECEF-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECEF-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECEF-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m) and resolved along
     *                     ECEF-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m) and resolved along
     *                     ECEF-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m) and resolved along
     *                     ECEF-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final double vx, final double vy, final double vz,
                                          final double oldVx, final double oldVy, final double oldVz,
                                          final double x, final double y, final double z,
                                          final ECEFKinematics result) {
        if (timeInterval < 0.0
                || !ECEFFrame.isValidCoordinateTransformation(c)
                || !ECEFFrame.isValidCoordinateTransformation(oldC)) {
            throw new IllegalArgumentException();
        }

        if (timeInterval > 0.0) {
            try {
                // From (2.145) determine the Earth rotation over the update interval
                final double alpha = EARTH_ROTATION_RATE * timeInterval;
                final Matrix cEarth = CoordinateTransformation.eciToEcefMatrixFromAngle(alpha);

                final Matrix cBe = c.getMatrix();
                final Matrix oldCbe = oldC.getMatrix();

                // cOldNew = cBe * cEarth * oldCbe
                cEarth.multiply(oldCbe);
                cBe.multiply(cEarth); // here cBe is cOldNew

                // Calculate the approximate angular rate with respect an intertial frame
                double alphaX = 0.5 * (cBe.getElementAt(1, 2)
                        - cBe.getElementAt(2, 1));
                double alphaY = 0.5 * (cBe.getElementAt(2, 0)
                        - cBe.getElementAt(0, 2));
                double alphaZ = 0.5 * (cBe.getElementAt(0, 1)
                        - cBe.getElementAt(1, 0));

                // Calculate and apply the scaling factor
                final double temp = Math.acos(0.5 * (cBe.getElementAt(0, 0)
                        + cBe.getElementAt(1, 1)
                        + cBe.getElementAt(2, 2) - 1.0));
                if (temp > SCALING_THRESHOLD) {
                    // scaling is 1 if temp is less than this
                    final double scale = temp / Math.sin(temp);
                    alphaX *= scale;
                    alphaY *= scale;
                    alphaZ *= scale;
                }

                // Calculate the angular rate
                final double angularRateX = alphaX / timeInterval;
                final double angularRateY = alphaY / timeInterval;
                final double angularRateZ = alphaZ / timeInterval;

                // Calculate the specific force resolved about ECEF-frame axes
                // Frame (5.36)
                final Matrix vEbe = new Matrix(ROWS, 1);
                vEbe.setElementAtIndex(0, vx);
                vEbe.setElementAtIndex(1, vy);
                vEbe.setElementAtIndex(2, vz);

                final Matrix oldVebe = new Matrix(ROWS, 1);
                oldVebe.setElementAtIndex(0, oldVx);
                oldVebe.setElementAtIndex(1, oldVy);
                oldVebe.setElementAtIndex(2, oldVz);

                final ECEFGravity gravity = ECEFGravityEstimator
                        .estimateGravityAndReturnNew(x, y, z);
                final Matrix g = gravity.asMatrix();

                final Matrix earthRotationSkew = Utils.skewMatrix(
                        new double[]{0.0, 0.0, EARTH_ROTATION_RATE});
                earthRotationSkew.multiplyByScalar(2.0);
                earthRotationSkew.multiply(oldVebe); // 2.0 * earthRotationSkew * oldVebe

                vEbe.subtract(oldVebe); // vEbe - oldVebe
                vEbe.multiplyByScalar(1.0 / timeInterval); // (vEbe - oldVebe) / timeInterval
                vEbe.subtract(g); // (vEbe - oldVebe) / timeInterval - g
                // fibe = (vEbe - oldVebe) / timeInterval - g + 2.0 * earthRotationSkew * oldVebe
                vEbe.add(earthRotationSkew); // vEbe contains specific force resolved about ECEF-frame axes

                // Calculate the average body-to-ECEF-frame coordinate transformation
                // matrix over the update interval using (5,84) and (5.85)
                final double alphaNorm = Math.sqrt(alphaX * alphaX
                        + alphaY * alphaY + alphaZ * alphaZ);
                final Matrix alphaSkew1 = Utils.skewMatrix(
                        new double[]{alphaX, alphaY, alphaZ});

                if (alphaNorm > ALPHA_THRESHOLD) {
                    final double alphaNorm2 = alphaNorm * alphaNorm;
                    final double value1 = (1.0 - Math.cos(alphaNorm)) / alphaNorm2;
                    final double value2 = (1.0 - Math.sin(alphaNorm) / alphaNorm) / alphaNorm2;
                    final Matrix tmp1 = alphaSkew1.multiplyByScalarAndReturnNew(value1);
                    final Matrix tmp2 = alphaSkew1.multiplyByScalarAndReturnNew(value2);
                    tmp2.multiply(alphaSkew1);

                    final Matrix tmp3 = Matrix.identity(ROWS, ROWS);
                    tmp3.add(tmp1);
                    tmp3.add(tmp2);

                    oldCbe.multiply(tmp3);
                }

                final Matrix alphaSkew2 = Utils.skewMatrix(
                        new double[]{0.0, 0.0, alpha});
                alphaSkew2.multiplyByScalar(0.5);
                alphaSkew2.multiply(oldCbe); // 0.5 * alphaSkew2 * oldCbe

                oldCbe.subtract(alphaSkew2); // oldCbe - 0.5 * alphaSkew2 * oldCbe
                // oldCbe now contains the average body-to-ECEF-frame coordinate transformation

                // Transform specific force to body-frame resolving axes using (5.81)
                Matrix invAveCbe = Utils.inverse(oldCbe);
                invAveCbe.multiply(vEbe);

                final double specificForceX = invAveCbe.getElementAtIndex(0);
                final double specificForceY = invAveCbe.getElementAtIndex(1);
                final double specificForceZ = invAveCbe.getElementAtIndex(2);

                // save result data
                result.setSpecificForceCoordinates(specificForceX, specificForceY,
                        specificForceZ);
                result.setAngularRateCoordinates(angularRateX, angularRateY,
                        angularRateZ);

            } catch (final AlgebraException ignore) {
                // never happens
            }
        } else {
            // If time interval is zero, set angular rate and specific force to zero
            result.setSpecificForceCoordinates(0.0, 0.0, 0.0);
            result.setAngularRateCoordinates(0.0, 0.0, 0.0);
        }
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param c            boy-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param x            cartesian x coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final double vx, final double vy, final double vz,
                                          final double oldVx, final double oldVy, final double oldVz,
                                          final double x, final double y, final double z,
                                          final ECEFKinematics result) {
        estimateKinematics(TimeConverter.convert(timeInterval.getValue().doubleValue(),
                timeInterval.getUnit(), TimeUnit.SECOND),
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final ECEFFrame frame,
                                          final CoordinateTransformation oldC,
                                          final double oldVx, final double oldVy, final double oldVz,
                                          final ECEFKinematics result) {
        estimateKinematics(timeInterval, frame.getCoordinateTransformation(),
                oldC, frame.getVx(), frame.getVy(), frame.getVz(),
                oldVx, oldVy, oldVz, frame.getX(), frame.getY(), frame.getZ(), result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final ECEFFrame frame,
                                          final CoordinateTransformation oldC,
                                          final double oldVx, final double oldVy, final double oldVz,
                                          final ECEFKinematics result) {
        estimateKinematics(TimeConverter.convert(timeInterval.getValue().doubleValue(),
                timeInterval.getUnit(), TimeUnit.SECOND),
                frame.getCoordinateTransformation(),
                oldC, frame.getVx(), frame.getVy(), frame.getVz(),
                oldVx, oldVy, oldVz, frame.getX(), frame.getY(), frame.getZ(), result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldFrame     body ECEF frame containing previous position, velocity and
     *                     body-to-ECEF frame coordinate transformation. Notice that
     *                     previous position contained in this frame is ignored.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final ECEFFrame frame, final ECEFFrame oldFrame,
                                          final ECEFKinematics result) {
        estimateKinematics(timeInterval, frame,
                oldFrame.getCoordinateTransformation(),
                oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(), result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldFrame     body ECEF frame containing previous position, velocity and
     *                     body-to-ECEF frame coordinate transformation. Notice that
     *                     previous position contained in this frame is ignored.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final ECEFFrame frame, final ECEFFrame oldFrame,
                                          final ECEFKinematics result) {
        estimateKinematics(timeInterval, frame,
                oldFrame.getCoordinateTransformation(),
                oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(), result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final Speed vx, final Speed vy, final Speed vz,
                                          final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                          final double x, final double y, final double z,
                                          final ECEFKinematics result) {
        estimateKinematics(timeInterval, c, oldC,
                SpeedConverter.convert(vx.getValue().doubleValue(), vx.getUnit(), SpeedUnit.METERS_PER_SECOND),
                SpeedConverter.convert(vy.getValue().doubleValue(), vy.getUnit(), SpeedUnit.METERS_PER_SECOND),
                SpeedConverter.convert(vz.getValue().doubleValue(), vz.getUnit(), SpeedUnit.METERS_PER_SECOND),
                SpeedConverter.convert(oldVx.getValue().doubleValue(), oldVx.getUnit(), SpeedUnit.METERS_PER_SECOND),
                SpeedConverter.convert(oldVy.getValue().doubleValue(), oldVy.getUnit(), SpeedUnit.METERS_PER_SECOND),
                SpeedConverter.convert(oldVz.getValue().doubleValue(), oldVz.getUnit(), SpeedUnit.METERS_PER_SECOND),
                x, y, z, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final Speed vx, final Speed vy, final Speed vz,
                                          final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                          final double x, final double y, final double z,
                                          final ECEFKinematics result) {
        estimateKinematics(timeInterval, c, oldC,
                SpeedConverter.convert(vx.getValue().doubleValue(), vx.getUnit(), SpeedUnit.METERS_PER_SECOND),
                SpeedConverter.convert(vy.getValue().doubleValue(), vy.getUnit(), SpeedUnit.METERS_PER_SECOND),
                SpeedConverter.convert(vz.getValue().doubleValue(), vz.getUnit(), SpeedUnit.METERS_PER_SECOND),
                SpeedConverter.convert(oldVx.getValue().doubleValue(), oldVx.getUnit(), SpeedUnit.METERS_PER_SECOND),
                SpeedConverter.convert(oldVy.getValue().doubleValue(), oldVy.getUnit(), SpeedUnit.METERS_PER_SECOND),
                SpeedConverter.convert(oldVz.getValue().doubleValue(), oldVz.getUnit(), SpeedUnit.METERS_PER_SECOND),
                x, y, z, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final ECEFFrame frame,
                                          final CoordinateTransformation oldC,
                                          final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                          final ECEFKinematics result) {
        estimateKinematics(timeInterval, frame, oldC,
                SpeedConverter.convert(oldVx.getValue().doubleValue(), oldVx.getUnit(), SpeedUnit.METERS_PER_SECOND),
                SpeedConverter.convert(oldVy.getValue().doubleValue(), oldVy.getUnit(), SpeedUnit.METERS_PER_SECOND),
                SpeedConverter.convert(oldVz.getValue().doubleValue(), oldVz.getUnit(), SpeedUnit.METERS_PER_SECOND),
                result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final ECEFFrame frame,
                                          final CoordinateTransformation oldC,
                                          final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                          final ECEFKinematics result) {
        estimateKinematics(timeInterval, frame, oldC,
                SpeedConverter.convert(oldVx.getValue().doubleValue(), oldVx.getUnit(), SpeedUnit.METERS_PER_SECOND),
                SpeedConverter.convert(oldVy.getValue().doubleValue(), oldVy.getUnit(), SpeedUnit.METERS_PER_SECOND),
                SpeedConverter.convert(oldVz.getValue().doubleValue(), oldVz.getUnit(), SpeedUnit.METERS_PER_SECOND),
                result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param position     body position expressed in meters (m) with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final double vx, final double vy, final double vz,
                                          final double oldVx, final double oldVy, final double oldVz,
                                          final Point3D position, final ECEFKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position.getInhomX(), position.getInhomY(), position.getInhomZ(), result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param position     body position expressed in meters (m) with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final double vx, final double vy, final double vz,
                                          final double oldVx, final double oldVy, final double oldVz,
                                          final Point3D position, final ECEFKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position.getInhomX(), position.getInhomY(), position.getInhomZ(), result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            boy-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @param position     body position expressed in meters (m) with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final Speed vx, final Speed vy, final Speed vz,
                                          final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                          final Point3D position, final ECEFKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position.getInhomX(), position.getInhomY(), position.getInhomZ(), result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param c            boy-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @param position     body position expressed in meters (m) with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final Speed vx, final Speed vy, final Speed vz,
                                          final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                          final Point3D position, final ECEFKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position.getInhomX(), position.getInhomY(), position.getInhomZ(), result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param x            cartesian x coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static ECEFKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final double vx, final double vy, final double vz,
                                                                final double oldVx, final double oldVy, final double oldVz,
                                                                final double x, final double y, final double z) {
        final ECEFKinematics result = new ECEFKinematics();
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between.
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param x            cartesian x coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static ECEFKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final double vx, final double vy, final double vz,
                                                                final double oldVx, final double oldVy, final double oldVz,
                                                                final double x, final double y, final double z) {
        final ECEFKinematics result = new ECEFKinematics();
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static ECEFKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final ECEFFrame frame,
                                                                final CoordinateTransformation oldC,
                                                                final double oldVx, final double oldVy,
                                                                final double oldVz) {
        final ECEFKinematics result = new ECEFKinematics();
        estimateKinematics(timeInterval, frame, oldC, oldVx, oldVy, oldVz, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static ECEFKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final ECEFFrame frame,
                                                                final CoordinateTransformation oldC,
                                                                final double oldVx, final double oldVy,
                                                                final double oldVz) {
        final ECEFKinematics result = new ECEFKinematics();
        estimateKinematics(timeInterval, frame, oldC, oldVx, oldVy, oldVz, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldFrame     body ECEF frame containing previous position, velocity and
     *                     body-to-ECEF frame coordinate transformation. Notice that
     *                     previous position contained in this frame is ignored.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static ECEFKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final ECEFFrame frame, final ECEFFrame oldFrame) {
        final ECEFKinematics result = new ECEFKinematics();
        estimateKinematics(timeInterval, frame, oldFrame, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldFrame     body ECEF frame containing previous position, velocity and
     *                     body-to-ECEF frame coordinate transformation. Notice that
     *                     previous position contained in this frame is ignored.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static ECEFKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final ECEFFrame frame, final ECEFFrame oldFrame) {
        final ECEFKinematics result = new ECEFKinematics();
        estimateKinematics(timeInterval, frame, oldFrame, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static ECEFKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final Speed vx, final Speed vy, final Speed vz,
                                                                final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                                                final double x, final double y, final double z) {
        final ECEFKinematics result = new ECEFKinematics();
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m)
     *                     with respect ECEF frame, resolved along ECEF-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static ECEFKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final Speed vx, final Speed vy, final Speed vz,
                                                                final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                                                final double x, final double y, final double z) {
        final ECEFKinematics result = new ECEFKinematics();
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static ECEFKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final ECEFFrame frame,
                                                                final CoordinateTransformation oldC,
                                                                final Speed oldVx, final Speed oldVy, final Speed oldVz) {
        final ECEFKinematics result = new ECEFKinematics();
        estimateKinematics(timeInterval, frame, oldC, oldVx, oldVy, oldVz, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECEF frame containing current position, velocity and
     *                     body-to-ECEF frame coordinate transformation.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static ECEFKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final ECEFFrame frame,
                                                                final CoordinateTransformation oldC,
                                                                final Speed oldVx, final Speed oldVy, final Speed oldVz) {
        final ECEFKinematics result = new ECEFKinematics();
        estimateKinematics(timeInterval, frame, oldC, oldVx, oldVy, oldVz, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param position     body position expressed in meters (m) with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static ECEFKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final double vx, final double vy, final double vz,
                                                                final double oldVx, final double oldVy, final double oldVz,
                                                                final Point3D position) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz,
                oldVx, oldVy, oldVz,
                position.getInhomX(), position.getInhomY(), position.getInhomZ());
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes and expressed in meters per
     *                     second (m/s).
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes and expressed in meters
     *                     per second (m/s).
     * @param position     body position expressed in meters (m) with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static ECEFKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final double vx, final double vy, final double vz,
                                                                final double oldVx, final double oldVy, final double oldVz,
                                                                final Point3D position) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position.getInhomX(), position.getInhomY(), position.getInhomZ());
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            boy-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @param position     body position expressed in meters (m) with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static ECEFKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final Speed vx, final Speed vy, final Speed vz,
                                                                final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                                                final Point3D position) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position.getInhomX(), position.getInhomY(), position.getInhomZ());
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param c            boy-to-ECEF frame coordinate transformation matrix.
     * @param oldC         previous body-to-ECEF-frame coordinate transformation matrix.
     * @param vx           velocity of body frame x coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vy           velocity of body frame y coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param vz           velocity of body frame z coordinate with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @param oldVx        previous velocity of body frame x coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVy        previous velocity of body frame y coordinate with respect ECEF
     *                     frame, resolved along ECEF-frame axes.
     * @param oldVz        previous velocity of body frame z coordinate with respect ECEF
     *                     frame, resolved along EVEF-frame axes.
     * @param position     body position expressed in meters (m) with respect ECEF frame,
     *                     resolved along ECEF-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinated transformation matrices
     *                                  are not ECEF frame valid.
     */
    public static ECEFKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                                final CoordinateTransformation c,
                                                                final CoordinateTransformation oldC,
                                                                final Speed vx, final Speed vy, final Speed vz,
                                                                final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                                                final Point3D position) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position.getInhomX(), position.getInhomY(), position.getInhomZ());
    }
}
