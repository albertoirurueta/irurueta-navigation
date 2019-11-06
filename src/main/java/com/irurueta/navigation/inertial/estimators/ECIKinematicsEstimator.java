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
import com.irurueta.navigation.frames.ECIFrame;
import com.irurueta.navigation.inertial.ECIGravitation;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedConverter;
import com.irurueta.units.SpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

/**
 * Estimates body kinematics (specific force applied to a body and its angular rates) with respect and resolved
 * along ECI-frame axes.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/MATLAB-Codes
 */
public class ECIKinematicsEstimator {

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
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final double vx, final double vy, final double vz,
                         final double oldVx, final double oldVy, final double oldVz,
                         final double x, final double y, final double z,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final double vx, final double vy, final double vz,
                         final double oldVx, final double oldVy, final double oldVz,
                         final double x, final double y, final double z,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public void estimate(final double timeInterval,
                         final ECIFrame frame,
                         final CoordinateTransformation oldC,
                         final double oldVx, final double oldVy, final double oldVz,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVx, oldVy, oldVz, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public void estimate(final Time timeInterval,
                         final ECIFrame frame,
                         final CoordinateTransformation oldC,
                         final double oldVx, final double oldVy, final double oldVz,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVx, oldVy, oldVz, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldFrame     body ECI frame containing previous position, velocity and
     *                     body-to-ECI frame coordinate transformation. Notice that
     *                     previous position contained in this frame is ignored.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final double timeInterval,
                         final ECIFrame frame, final ECIFrame oldFrame,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldFrame     body ECI frame containing previous position, velocity and
     *                     body-to-ECI frame coordinate transformation. Notice that
     *                     previous position contained in this frame is ignored.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public void estimate(final Time timeInterval,
                         final ECIFrame frame, final ECIFrame oldFrame,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldFrame, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final Speed vx, final Speed vy, final Speed vz,
                         final Speed oldVx, final Speed oldVy, final Speed oldVz,
                         final double x, final double y, final double z,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final Speed vx, final Speed vy, final Speed vz,
                         final Speed oldVx, final Speed oldVy, final Speed oldVz,
                         final double x, final double y, final double z,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        x coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public void estimate(final double timeInterval,
                         final ECIFrame frame,
                         final CoordinateTransformation oldC,
                         final Speed oldVx, final Speed oldVy, final Speed oldVz,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVx, oldVy, oldVz, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        x coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public void estimate(final Time timeInterval,
                         final ECIFrame frame,
                         final CoordinateTransformation oldC,
                         final Speed oldVx, final Speed oldVy, final Speed oldVz,
                         final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC, oldVx, oldVy, oldVz, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param position     body position expressed in meters (m) with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final double vx, final double vy, final double vz,
                         final double oldVx, final double oldVy, final double oldVz,
                         final Point3D position, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param position     body position expressed in meters (m) with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final double vx, final double vy, final double vz,
                         final double oldVx, final double oldVy, final double oldVz,
                         final Point3D position, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param position     body position expressed in meters (m) with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public void estimate(final double timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final Speed vx, final Speed vy, final Speed vz,
                         final Speed oldVx, final Speed oldVy, final Speed oldVz,
                         final Point3D position, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param position     body position expressed in meters (m) with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public void estimate(final Time timeInterval,
                         final CoordinateTransformation c,
                         final CoordinateTransformation oldC,
                         final Speed vx, final Speed vy, final Speed vz,
                         final Speed oldVx, final Speed oldVy, final Speed oldVz,
                         final Point3D position, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                              final CoordinateTransformation c,
                                              final CoordinateTransformation oldC,
                                              final double vx, final double vy, final double vz,
                                              final double oldVx, final double oldVy, final double oldVz,
                                              final double x, final double y, final double z) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                              final CoordinateTransformation c,
                                              final CoordinateTransformation oldC,
                                              final double vx, final double vy, final double vz,
                                              final double oldVx, final double oldVy, final double oldVz,
                                              final double x, final double y, final double z) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                              final ECIFrame frame,
                                              final CoordinateTransformation oldC,
                                              final double oldVx, final double oldVy, final double oldVz) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldC, oldVx, oldVy, oldVz);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                              final ECIFrame frame,
                                              final CoordinateTransformation oldC,
                                              final double oldVx, final double oldVy, final double oldVz) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldC, oldVx, oldVy, oldVz);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldFrame     body ECI frame containing previous position, velocity and
     *                     body-to-ECI frame coordinate transformation. Notice that
     *                     previous position contained in this frame is ignored.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                              final ECIFrame frame, final ECIFrame oldFrame) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldFrame);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldFrame     body ECI frame containing previous position, velocity and
     *                     body-to-ECI frame coordinate transformation. Notice that
     *                     previous position contained in this frame is ignored.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                              final ECIFrame frame, final ECIFrame oldFrame) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldFrame);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                              final CoordinateTransformation c,
                                              final CoordinateTransformation oldC,
                                              final Speed vx, final Speed vy, final Speed vz,
                                              final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                              final double x, final double y, final double z) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                              final CoordinateTransformation c,
                                              final CoordinateTransformation oldC,
                                              final Speed vx, final Speed vy, final Speed vz,
                                              final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                              final double x, final double y, final double z) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        x coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                              final ECIFrame frame,
                                              final CoordinateTransformation oldC,
                                              final Speed oldVx, final Speed oldVy, final Speed oldVz) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldC, oldVx, oldVy, oldVz);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        x coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                              final ECIFrame frame,
                                              final CoordinateTransformation oldC,
                                              final Speed oldVx, final Speed oldVy, final Speed oldVz) {
        return estimateKinematicsAndReturnNew(timeInterval, frame, oldC, oldVx, oldVy, oldVz);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param position     body position expressed in meters (m) with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                              final CoordinateTransformation c,
                                              final CoordinateTransformation oldC,
                                              final double vx, final double vy, final double vz,
                                              final double oldVx, final double oldVy, final double oldVz,
                                              final Point3D position) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param position     body position expressed in meters (m) with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                              final CoordinateTransformation c,
                                              final CoordinateTransformation oldC,
                                              final double vx, final double vy, final double vz,
                                              final double oldVx, final double oldVy, final double oldVz,
                                              final Point3D position) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param position     body position expressed in meters (m) with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final double timeInterval,
                                              final CoordinateTransformation c,
                                              final CoordinateTransformation oldC,
                                              final Speed vx, final Speed vy, final Speed vz,
                                              final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                              final Point3D position) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param position     body position expressed in meters (m) with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public BodyKinematics estimateAndReturnNew(final Time timeInterval,
                                              final CoordinateTransformation c,
                                              final CoordinateTransformation oldC,
                                              final Speed vx, final Speed vy, final Speed vz,
                                              final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                              final Point3D position) {
        return estimateKinematicsAndReturnNew(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final double vx, final double vy, final double vz,
                                          final double oldVx, final double oldVy, final double oldVz,
                                          final double x, final double y, final double z,
                                          final BodyKinematics result) {
        if (timeInterval < 0.0
                || !ECIFrame.isValidCoordinateTransformation(c)
                || !ECIFrame.isValidCoordinateTransformation(oldC)) {
            throw new IllegalArgumentException();
        }

        if (timeInterval > 0.0) {
            try {
                // Obtain coordinate transformation matrix from the old attitude to the new
                // initially c contains body-to-ECI transformation
                final Matrix cib = c.getMatrix();
                // we transpose it to obtain the inverse
                cib.transpose();

                final Matrix oldCbi = oldC.getMatrix();
                final Matrix cOldNew = cib.multiplyAndReturnNew(oldCbi);

                // Calculate the approximate angular rate
                double alphaIbbX = 0.5 * (cOldNew.getElementAt(1, 2)
                        - cOldNew.getElementAt(2, 1));
                double alphaIbbY = 0.5 * (cOldNew.getElementAt(2, 0)
                        - cOldNew.getElementAt(0, 2));
                double alphaIbbZ = 0.5 * (cOldNew.getElementAt(0, 1)
                        - cOldNew.getElementAt(1, 0));

                // Calculate and apply the scaling factor
                final double temp = Math.acos(0.5 * (cOldNew.getElementAt(0, 0)
                        + cOldNew.getElementAt(1, 1)
                        + cOldNew.getElementAt(2, 2) - 1.0));
                if (temp > SCALING_THRESHOLD) {
                    // Scaling is 1 if temp is less than this
                    final double scaling = temp / Math.sin(temp);
                    alphaIbbX *= scaling;
                    alphaIbbY *= scaling;
                    alphaIbbZ *= scaling;
                }

                // Calculate the angular rate
                final double angularRateX = alphaIbbX / timeInterval;
                final double angularRateY = alphaIbbY / timeInterval;
                final double angularRateZ = alphaIbbZ / timeInterval;

                // Calculate the specific force resolved about ECI-frame axes
                // From (5.18) and (5.20)
                final Matrix vIbi = new Matrix(ROWS, 1);
                vIbi.setElementAtIndex(0, vx);
                vIbi.setElementAtIndex(1, vy);
                vIbi.setElementAtIndex(2, vz);

                final Matrix oldVibi = new Matrix(ROWS, 1);
                oldVibi.setElementAtIndex(0, oldVx);
                oldVibi.setElementAtIndex(1, oldVy);
                oldVibi.setElementAtIndex(2, oldVz);

                final ECIGravitation gravitation = ECIGravitationEstimator
                        .estimateGravitationAndReturnNew(x, y, z);
                final Matrix g = gravitation.asMatrix();

                // fIbi = ((vIbi - oldVibi) / timeInterval) - g
                vIbi.subtract(oldVibi);
                vIbi.multiplyByScalar(1.0 / timeInterval);
                vIbi.subtract(g);
                // vIbi now contains specific force resolved about ECI-frame axes

                // Calculate the average body-to-ECI-frame coordinate transformation
                // matrix over the update interval using (5.84)
                final double alphaNorm = Math.sqrt(alphaIbbX * alphaIbbX +
                        alphaIbbY * alphaIbbY + alphaIbbZ * alphaIbbZ);
                final Matrix alphaSkew = Utils.skewMatrix(new double[]{alphaIbbX, alphaIbbY, alphaIbbZ});

                if (alphaNorm > ALPHA_THRESHOLD) {
                    final double alphaNorm2 = alphaNorm * alphaNorm;
                    final double value1 = (1.0 - Math.cos(alphaNorm)) / alphaNorm2;
                    final double value2 = (1.0 - Math.sin(alphaNorm) / alphaNorm) / alphaNorm2;
                    final Matrix tmp1 = alphaSkew.multiplyByScalarAndReturnNew(value1);
                    final Matrix tmp2 = alphaSkew.multiplyByScalarAndReturnNew(value2);
                    tmp2.multiply(alphaSkew);

                    final Matrix tmp3 = Matrix.identity(ROWS, ROWS);
                    tmp3.add(tmp1);
                    tmp3.add(tmp2);

                    oldCbi.multiply(tmp3);
                }

                // oldCbi now contains the average body-to-ECI-frame coordinate transformation

                // Transform specific force to body-frame resolving axes using (5.81)
                Matrix invAveCbi = Utils.inverse(oldCbi);
                invAveCbi.multiply(vIbi);

                final double specificForceX = invAveCbi.getElementAtIndex(0);
                final double specificForceY = invAveCbi.getElementAtIndex(1);
                final double specificForceZ = invAveCbi.getElementAtIndex(2);

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
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final double vx, final double vy, final double vz,
                                          final double oldVx, final double oldVy, final double oldVz,
                                          final double x, final double y, final double z,
                                          final BodyKinematics result) {
        estimateKinematics(TimeConverter.convert(timeInterval.getValue().doubleValue(),
                timeInterval.getUnit(), TimeUnit.SECOND),
                c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, x, y, z, result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final ECIFrame frame,
                                          final CoordinateTransformation oldC,
                                          final double oldVx, final double oldVy, final double oldVz,
                                          final BodyKinematics result) {
        estimateKinematics(timeInterval, frame.getCoordinateTransformation(),
                oldC, frame.getVx(), frame.getVy(), frame.getVz(),
                oldVx, oldVy, oldVz, frame.getX(), frame.getY(), frame.getZ(), result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final ECIFrame frame,
                                          final CoordinateTransformation oldC,
                                          final double oldVx, final double oldVy, final double oldVz,
                                          final BodyKinematics result) {
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
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldFrame     body ECI frame containing previous position, velocity and
     *                     body-to-ECI frame coordinate transformation. Notice that
     *                     previous position contained in this frame is ignored.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final ECIFrame frame, final ECIFrame oldFrame,
                                          final BodyKinematics result) {
        estimateKinematics(timeInterval, frame,
                oldFrame.getCoordinateTransformation(),
                oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(), result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldFrame     body ECI frame containing previous position, velocity and
     *                     body-to-ECI frame coordinate transformation. Notice that
     *                     previous position contained in this frame is ignored.
     * @param result       instance where body kinematics estimation will be stored.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final ECIFrame frame, final ECIFrame oldFrame,
                                          final BodyKinematics result) {
        estimateKinematics(timeInterval, frame,
                oldFrame.getCoordinateTransformation(),
                oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(), result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final Speed vx, final Speed vy, final Speed vz,
                                          final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                          final double x, final double y, final double z,
                                          final BodyKinematics result) {
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
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final Speed vx, final Speed vy, final Speed vz,
                                          final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                          final double x, final double y, final double z,
                                          final BodyKinematics result) {
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
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        x coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final ECIFrame frame,
                                          final CoordinateTransformation oldC,
                                          final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                          final BodyKinematics result) {
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
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        x coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final ECIFrame frame,
                                          final CoordinateTransformation oldC,
                                          final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                          final BodyKinematics result) {
        estimateKinematics(timeInterval, frame, oldC,
                SpeedConverter.convert(oldVx.getValue().doubleValue(), oldVx.getUnit(), SpeedUnit.METERS_PER_SECOND),
                SpeedConverter.convert(oldVy.getValue().doubleValue(), oldVy.getUnit(), SpeedUnit.METERS_PER_SECOND),
                SpeedConverter.convert(oldVz.getValue().doubleValue(), oldVz.getUnit(), SpeedUnit.METERS_PER_SECOND),
                result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param position     body position expressed in meters (m) with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final double vx, final double vy, final double vz,
                                          final double oldVx, final double oldVy, final double oldVz,
                                          final Point3D position, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position.getInhomX(), position.getInhomY(), position.getInhomZ(), result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param position     body position expressed in meters (m) with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final double vx, final double vy, final double vz,
                                          final double oldVx, final double oldVy, final double oldVz,
                                          final Point3D position, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position.getInhomX(), position.getInhomY(), position.getInhomZ(), result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param position     body position expressed in meters (m) with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static void estimateKinematics(final double timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final Speed vx, final Speed vy, final Speed vz,
                                          final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                          final Point3D position, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position.getInhomX(), position.getInhomY(), position.getInhomZ(), result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param position     body position expressed in meters (m) with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param result       instance where estimated body kinematics will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static void estimateKinematics(final Time timeInterval,
                                          final CoordinateTransformation c,
                                          final CoordinateTransformation oldC,
                                          final Speed vx, final Speed vy, final Speed vz,
                                          final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                          final Point3D position, final BodyKinematics result) {
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                position.getInhomX(), position.getInhomY(), position.getInhomZ(), result);
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                               final CoordinateTransformation c,
                                                               final CoordinateTransformation oldC,
                                                               final double vx, final double vy, final double vz,
                                                               final double oldVx, final double oldVy, final double oldVz,
                                                               final double x, final double y, final double z) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                               final CoordinateTransformation c,
                                                               final CoordinateTransformation oldC,
                                                               final double vx, final double vy, final double vz,
                                                               final double oldVx, final double oldVy, final double oldVz,
                                                               final double x, final double y, final double z) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                               final ECIFrame frame,
                                                               final CoordinateTransformation oldC,
                                                               final double oldVx, final double oldVy, final double oldVz) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, frame, oldC, oldVx, oldVy, oldVz, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                               final ECIFrame frame,
                                                               final CoordinateTransformation oldC,
                                                               final double oldVx, final double oldVy, final double oldVz) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, frame, oldC, oldVx, oldVy, oldVz, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldFrame     body ECI frame containing previous position, velocity and
     *                     body-to-ECI frame coordinate transformation. Notice that
     *                     previous position contained in this frame is ignored.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                               final ECIFrame frame, final ECIFrame oldFrame) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, frame, oldFrame, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldFrame     body ECI frame containing previous position, velocity and
     *                     body-to-ECI frame coordinate transformation. Notice that
     *                     previous position contained in this frame is ignored.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                               final ECIFrame frame, final ECIFrame oldFrame) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, frame, oldFrame, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                               final CoordinateTransformation c,
                                                               final CoordinateTransformation oldC,
                                                               final Speed vx, final Speed vy, final Speed vz,
                                                               final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                                               final double x, final double y, final double z) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param x            cartesian x coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param y            cartesian y coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @param z            cartesian z coordinate of body position expressed in meters (m) and resolved along
     *                     ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                               final CoordinateTransformation c,
                                                               final CoordinateTransformation oldC,
                                                               final Speed vx, final Speed vy, final Speed vz,
                                                               final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                                               final double x, final double y, final double z) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz,
                x, y, z, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        x coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                               final ECIFrame frame,
                                                               final CoordinateTransformation oldC,
                                                               final Speed oldVx, final Speed oldVy, final Speed oldVz) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, frame, oldC, oldVx, oldVy, oldVz, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates).
     *
     * @param timeInterval time interval between epochs.
     * @param frame        body ECI frame containing current position, velocity and
     *                     body-to-ECI frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        x coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                               final ECIFrame frame,
                                                               final CoordinateTransformation oldC,
                                                               final Speed oldVx, final Speed oldVy, final Speed oldVz) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, frame, oldC, oldVx, oldVy, oldVz, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param position     body position expressed in meters (m) with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                               final CoordinateTransformation c,
                                                               final CoordinateTransformation oldC,
                                                               final double vx, final double vy, final double vz,
                                                               final double oldVx, final double oldVy, final double oldVz,
                                                               final Point3D position) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame expressed in meters per second (m/s) and resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame expressed in meters per second (m/s) and
     *                     resolved along ECI-frame axes.
     * @param position     body position expressed in meters (m) with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                               final CoordinateTransformation c,
                                                               final CoordinateTransformation oldC,
                                                               final double vx, final double vy, final double vz,
                                                               final double oldVx, final double oldVy, final double oldVz,
                                                               final Point3D position) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param position     body position expressed in meters (m) with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final double timeInterval,
                                                               final CoordinateTransformation c,
                                                               final CoordinateTransformation oldC,
                                                               final Speed vx, final Speed vy, final Speed vz,
                                                               final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                                               final Point3D position) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position, result);
        return result;
    }

    /**
     * Estimates body kinematics (specific force applied to a body and its angular rates)..
     *
     * @param timeInterval time interval between epochs.
     * @param c            body-to-ECI-frame coordinate transformation.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param vx           x coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vy           y coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param vz           z coordinate of velocity of body frame with respect ECI frame, resolved
     *                     along ECI-frame axes.
     * @param oldVx        x coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVy        y coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldVz        z coordinate of previous velocity of body frame respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param position     body position expressed in meters (m) with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @return a new body kinematics instance.
     * @throws IllegalArgumentException if provided time interval is negative or coordinates transformation matrices
     *                                  are not ECI frame valid.
     */
    public static BodyKinematics estimateKinematicsAndReturnNew(final Time timeInterval,
                                                               final CoordinateTransformation c,
                                                               final CoordinateTransformation oldC,
                                                               final Speed vx, final Speed vy, final Speed vz,
                                                               final Speed oldVx, final Speed oldVy, final Speed oldVz,
                                                               final Point3D position) {
        final BodyKinematics result = new BodyKinematics();
        estimateKinematics(timeInterval, c, oldC, vx, vy, vz, oldVx, oldVy, oldVz, position, result);
        return result;
    }
}
