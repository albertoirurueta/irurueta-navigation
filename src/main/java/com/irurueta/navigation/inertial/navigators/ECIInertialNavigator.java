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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.Utils;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECIFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.ECIGravitation;
import com.irurueta.navigation.inertial.estimators.ECIGravitationEstimator;
import com.irurueta.units.*;

/**
 * Runs precision ECI-frame inertial navigation equations.
 * This implementation is based on the equations defined in "Principles of GNSS, Inertial, and Multisensor
 * Integrated Navigation Systems, Second Edition" and on the companion software available at:
 * https://github.com/ymjdz/MATLAB-Codes/blob/master/Nav_equations_ECI.m
 */
public class ECIInertialNavigator {

    /**
     * Alpha threshold.
     */
    private static final double ALPHA_THRESHOLD = 1e-8;

    /**
     * Number of rows.
     */
    private static final int ROWS = 3;

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public void navigate(final Time timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public void navigate(final double timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final BodyKinematics kinematics,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, kinematics,
                result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public void navigate(final Time timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final BodyKinematics kinematics,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, kinematics,
                result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous cartesian position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public void navigate(final double timeInterval,
                         final Point3D oldPosition,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous cartesian position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public void navigate(final Time timeInterval,
                         final Point3D oldPosition,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous cartesian position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public void navigate(final double timeInterval,
                         final Point3D oldPosition,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final BodyKinematics kinematics,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz, kinematics,
                result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous cartesian position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public void navigate(final Time timeInterval,
                         final Point3D oldPosition,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final BodyKinematics kinematics,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz, kinematics,
                result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final Distance oldX,
                         final Distance oldY,
                         final Distance oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final Distance oldX,
                         final Distance oldY,
                         final Distance oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public void navigate(final double timeInterval,
                         final Distance oldX,
                         final Distance oldY,
                         final Distance oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final BodyKinematics kinematics,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public void navigate(final Time timeInterval,
                         final Distance oldX,
                         final Distance oldY,
                         final Distance oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final BodyKinematics kinematics,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final Speed oldSpeedX,
                         final Speed oldSpeedY,
                         final Speed oldSpeedZ,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final Speed oldSpeedX,
                         final Speed oldSpeedY,
                         final Speed oldSpeedZ,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public void navigate(final double timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final Speed oldSpeedX,
                         final Speed oldSpeedY,
                         final Speed oldSpeedZ,
                         final BodyKinematics kinematics,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public void navigate(final Time timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final Speed oldSpeedX,
                         final Speed oldSpeedY,
                         final Speed oldSpeedZ,
                         final BodyKinematics kinematics,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final Acceleration fx,
                         final Acceleration fy,
                         final Acceleration fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final Acceleration fx,
                         final Acceleration fy,
                         final Acceleration fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final double fx,
                         final double fy,
                         final double fz,
                         final AngularSpeed angularRateX,
                         final AngularSpeed angularRateY,
                         final AngularSpeed angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final double fx,
                         final double fy,
                         final double fz,
                         final AngularSpeed angularRateX,
                         final AngularSpeed angularRateY,
                         final AngularSpeed angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final Distance oldX,
                         final Distance oldY,
                         final Distance oldZ,
                         final CoordinateTransformation oldC,
                         final Speed oldSpeedX,
                         final Speed oldSpeedY,
                         final Speed oldSpeedZ,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final Distance oldX,
                         final Distance oldY,
                         final Distance oldZ,
                         final CoordinateTransformation oldC,
                         final Speed oldSpeedX,
                         final Speed oldSpeedY,
                         final Speed oldSpeedZ,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final Distance oldX,
                         final Distance oldY,
                         final Distance oldZ,
                         final CoordinateTransformation oldC,
                         final Speed oldSpeedX,
                         final Speed oldSpeedY,
                         final Speed oldSpeedZ,
                         final Acceleration fx,
                         final Acceleration fy,
                         final Acceleration fz,
                         final AngularSpeed angularRateX,
                         final AngularSpeed angularRateY,
                         final AngularSpeed angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final Distance oldX,
                         final Distance oldY,
                         final Distance oldZ,
                         final CoordinateTransformation oldC,
                         final Speed oldSpeedX,
                         final Speed oldSpeedY,
                         final Speed oldSpeedZ,
                         final Acceleration fx,
                         final Acceleration fy,
                         final Acceleration fz,
                         final AngularSpeed angularRateX,
                         final AngularSpeed angularRateY,
                         final AngularSpeed angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final double timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final Acceleration fx,
                         final Acceleration fy,
                         final Acceleration fz,
                         final AngularSpeed angularRateX,
                         final AngularSpeed angularRateY,
                         final AngularSpeed angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public void navigate(final Time timeInterval,
                         final double oldX,
                         final double oldY,
                         final double oldZ,
                         final CoordinateTransformation oldC,
                         final double oldVx,
                         final double oldVy,
                         final double oldVz,
                         final Acceleration fx,
                         final Acceleration fy,
                         final Acceleration fz,
                         final AngularSpeed angularRateX,
                         final AngularSpeed angularRateY,
                         final AngularSpeed angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public void navigate(final double timeInterval,
                         final Distance oldX,
                         final Distance oldY,
                         final Distance oldZ,
                         final CoordinateTransformation oldC,
                         final Speed oldSpeedX,
                         final Speed oldSpeedY,
                         final Speed oldSpeedZ,
                         final BodyKinematics kinematics,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public void navigate(final Time timeInterval,
                         final Distance oldX,
                         final Distance oldY,
                         final Distance oldZ,
                         final CoordinateTransformation oldC,
                         final Speed oldSpeedX,
                         final Speed oldSpeedY,
                         final Speed oldSpeedZ,
                         final BodyKinematics kinematics,
                         final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public void navigate(final double timeInterval,
                         final ECIFrame oldFrame,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException {
        navigateECI(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public void navigate(final Time timeInterval,
                         final ECIFrame oldFrame,
                         final double fx,
                         final double fy,
                         final double fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException {
        navigateECI(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public void navigate(final double timeInterval,
                         final ECIFrame oldFrame,
                         final BodyKinematics kinematics,
                         final ECIFrame result)
            throws InertialNavigatorException {
        navigateECI(timeInterval, oldFrame, kinematics, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public void navigate(final Time timeInterval,
                         final ECIFrame oldFrame,
                         final BodyKinematics kinematics,
                         final ECIFrame result)
            throws InertialNavigatorException {
        navigateECI(timeInterval, oldFrame, kinematics, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public void navigate(final double timeInterval,
                         final ECIFrame oldFrame,
                         final Acceleration fx,
                         final Acceleration fy,
                         final Acceleration fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException {
        navigateECI(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public void navigate(final Time timeInterval,
                         final ECIFrame oldFrame,
                         final Acceleration fx,
                         final Acceleration fy,
                         final Acceleration fz,
                         final double angularRateX,
                         final double angularRateY,
                         final double angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException {
        navigateECI(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public void navigate(final double timeInterval,
                         final ECIFrame oldFrame,
                         final double fx,
                         final double fy,
                         final double fz,
                         final AngularSpeed angularRateX,
                         final AngularSpeed angularRateY,
                         final AngularSpeed angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException {
        navigateECI(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public void navigate(final Time timeInterval,
                         final ECIFrame oldFrame,
                         final double fx,
                         final double fy,
                         final double fz,
                         final AngularSpeed angularRateX,
                         final AngularSpeed angularRateY,
                         final AngularSpeed angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException {
        navigateECI(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public void navigate(final double timeInterval,
                         final ECIFrame oldFrame,
                         final Acceleration fx,
                         final Acceleration fy,
                         final Acceleration fz,
                         final AngularSpeed angularRateX,
                         final AngularSpeed angularRateY,
                         final AngularSpeed angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException {
        navigateECI(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public void navigate(final Time timeInterval,
                         final ECIFrame oldFrame,
                         final Acceleration fx,
                         final Acceleration fy,
                         final Acceleration fz,
                         final AngularSpeed angularRateX,
                         final AngularSpeed angularRateY,
                         final AngularSpeed angularRateZ,
                         final ECIFrame result)
            throws InertialNavigatorException {
        navigateECI(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public ECIFrame navigateAndReturnNew(final double timeInterval,
                                        final double oldX,
                                        final double oldY,
                                        final double oldZ,
                                        final CoordinateTransformation oldC,
                                        final double oldVx,
                                        final double oldVy,
                                        final double oldVz,
                                        final double fx,
                                        final double fy,
                                        final double fz,
                                        final double angularRateX,
                                        final double angularRateY,
                                        final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public ECIFrame navigateAndReturnNew(final Time timeInterval,
                                         final double oldX,
                                         final double oldY,
                                         final double oldZ,
                                         final CoordinateTransformation oldC,
                                         final double oldVx,
                                         final double oldVy,
                                         final double oldVz,
                                         final double fx,
                                         final double fy,
                                         final double fz,
                                         final double angularRateX,
                                         final double angularRateY,
                                         final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public ECIFrame navigateAndReturnNew(final double timeInterval,
                                         final double oldX,
                                         final double oldY,
                                         final double oldZ,
                                         final CoordinateTransformation oldC,
                                         final double oldVx,
                                         final double oldVy,
                                         final double oldVz,
                                         final BodyKinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, kinematics);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public ECIFrame navigateAndReturnNew(final Time timeInterval,
                                         final double oldX,
                                         final double oldY,
                                         final double oldZ,
                                         final CoordinateTransformation oldC,
                                         final double oldVx,
                                         final double oldVy,
                                         final double oldVz,
                                         final BodyKinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, kinematics);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous cartesian position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public ECIFrame navigateAndReturnNew(final double timeInterval,
                                         final Point3D oldPosition,
                                         final CoordinateTransformation oldC,
                                         final double oldVx,
                                         final double oldVy,
                                         final double oldVz,
                                         final double fx,
                                         final double fy,
                                         final double fz,
                                         final double angularRateX,
                                         final double angularRateY,
                                         final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldPosition, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous cartesian position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public ECIFrame navigateAndReturnNew(final Time timeInterval,
                                         final Point3D oldPosition,
                                         final CoordinateTransformation oldC,
                                         final double oldVx,
                                         final double oldVy,
                                         final double oldVz,
                                         final double fx,
                                         final double fy,
                                         final double fz,
                                         final double angularRateX,
                                         final double angularRateY,
                                         final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldPosition, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous cartesian position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public ECIFrame navigateAndReturnNew(final double timeInterval,
                                         final Point3D oldPosition,
                                         final CoordinateTransformation oldC,
                                         final double oldVx,
                                         final double oldVy,
                                         final double oldVz,
                                         final BodyKinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldPosition, oldC,
                oldVx, oldVy, oldVz, kinematics);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous cartesian position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public ECIFrame navigateAndReturnNew(final Time timeInterval,
                                         final Point3D oldPosition,
                                         final CoordinateTransformation oldC,
                                         final double oldVx,
                                         final double oldVy,
                                         final double oldVz,
                                         final BodyKinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldPosition, oldC,
                oldVx, oldVy, oldVz, kinematics);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public ECIFrame navigateAndReturnNew(final double timeInterval,
                                         final Distance oldX,
                                         final Distance oldY,
                                         final Distance oldZ,
                                         final CoordinateTransformation oldC,
                                         final double oldVx,
                                         final double oldVy,
                                         final double oldVz,
                                         final double fx,
                                         final double fy,
                                         final double fz,
                                         final double angularRateX,
                                         final double angularRateY,
                                         final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public ECIFrame navigateAndReturnNew(final Time timeInterval,
                                         final Distance oldX,
                                         final Distance oldY,
                                         final Distance oldZ,
                                         final CoordinateTransformation oldC,
                                         final double oldVx,
                                         final double oldVy,
                                         final double oldVz,
                                         final double fx,
                                         final double fy,
                                         final double fz,
                                         final double angularRateX,
                                         final double angularRateY,
                                         final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public ECIFrame navigateAndReturnNew(final double timeInterval,
                                         final Distance oldX,
                                         final Distance oldY,
                                         final Distance oldZ,
                                         final CoordinateTransformation oldC,
                                         final double oldVx,
                                         final double oldVy,
                                         final double oldVz,
                                         final BodyKinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, kinematics);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public ECIFrame navigateAndReturnNew(final Time timeInterval,
                                         final Distance oldX,
                                         final Distance oldY,
                                         final Distance oldZ,
                                         final CoordinateTransformation oldC,
                                         final double oldVx,
                                         final double oldVy,
                                         final double oldVz,
                                         final BodyKinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, kinematics);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public ECIFrame navigateAndReturnNew(final double timeInterval,
                                         final double oldX,
                                         final double oldY,
                                         final double oldZ,
                                         final CoordinateTransformation oldC,
                                         final Speed oldSpeedX,
                                         final Speed oldSpeedY,
                                         final Speed oldSpeedZ,
                                         final double fx,
                                         final double fy,
                                         final double fz,
                                         final double angularRateX,
                                         final double angularRateY,
                                         final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public ECIFrame navigateAndReturnNew(final Time timeInterval,
                                         final double oldX,
                                         final double oldY,
                                         final double oldZ,
                                         final CoordinateTransformation oldC,
                                         final Speed oldSpeedX,
                                         final Speed oldSpeedY,
                                         final Speed oldSpeedZ,
                                         final double fx,
                                         final double fy,
                                         final double fz,
                                         final double angularRateX,
                                         final double angularRateY,
                                         final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public ECIFrame navigateAndReturnNew(final double timeInterval,
                                         final double oldX,
                                         final double oldY,
                                         final double oldZ,
                                         final CoordinateTransformation oldC,
                                         final Speed oldSpeedX,
                                         final Speed oldSpeedY,
                                         final Speed oldSpeedZ,
                                         final BodyKinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public ECIFrame navigateAndReturnNew(final Time timeInterval,
                                         final double oldX,
                                         final double oldY,
                                         final double oldZ,
                                         final CoordinateTransformation oldC,
                                         final Speed oldSpeedX,
                                         final Speed oldSpeedY,
                                         final Speed oldSpeedZ,
                                         final BodyKinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public ECIFrame navigateAndReturnNew(final double timeInterval,
                                         final double oldX,
                                         final double oldY,
                                         final double oldZ,
                                         final CoordinateTransformation oldC,
                                         final double oldVx,
                                         final double oldVy,
                                         final double oldVz,
                                         final Acceleration fx,
                                         final Acceleration fy,
                                         final Acceleration fz,
                                         final double angularRateX,
                                         final double angularRateY,
                                         final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public ECIFrame navigateAndReturnNew(final Time timeInterval,
                                         final double oldX,
                                         final double oldY,
                                         final double oldZ,
                                         final CoordinateTransformation oldC,
                                         final double oldVx,
                                         final double oldVy,
                                         final double oldVz,
                                         final Acceleration fx,
                                         final Acceleration fy,
                                         final Acceleration fz,
                                         final double angularRateX,
                                         final double angularRateY,
                                         final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public ECIFrame navigateAndReturnNew(final double timeInterval,
                                         final double oldX,
                                         final double oldY,
                                         final double oldZ,
                                         final CoordinateTransformation oldC,
                                         final double oldVx,
                                         final double oldVy,
                                         final double oldVz,
                                         final double fx,
                                         final double fy,
                                         final double fz,
                                         final AngularSpeed angularRateX,
                                         final AngularSpeed angularRateY,
                                         final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ,
                oldC, oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public ECIFrame navigateAndReturnNew(final Time timeInterval,
                                         final double oldX,
                                         final double oldY,
                                         final double oldZ,
                                         final CoordinateTransformation oldC,
                                         final double oldVx,
                                         final double oldVy,
                                         final double oldVz,
                                         final double fx,
                                         final double fy,
                                         final double fz,
                                         final AngularSpeed angularRateX,
                                         final AngularSpeed angularRateY,
                                         final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public ECIFrame navigateAndReturnNew(final double timeInterval,
                                         final Distance oldX,
                                         final Distance oldY,
                                         final Distance oldZ,
                                         final CoordinateTransformation oldC,
                                         final Speed oldSpeedX,
                                         final Speed oldSpeedY,
                                         final Speed oldSpeedZ,
                                         final double fx,
                                         final double fy,
                                         final double fz,
                                         final double angularRateX,
                                         final double angularRateY,
                                         final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public ECIFrame navigateAndReturnNew(final Time timeInterval,
                                         final Distance oldX,
                                         final Distance oldY,
                                         final Distance oldZ,
                                         final CoordinateTransformation oldC,
                                         final Speed oldSpeedX,
                                         final Speed oldSpeedY,
                                         final Speed oldSpeedZ,
                                         final double fx,
                                         final double fy,
                                         final double fz,
                                         final double angularRateX,
                                         final double angularRateY,
                                         final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public ECIFrame navigateAndReturnNew(final double timeInterval,
                                         final Distance oldX,
                                         final Distance oldY,
                                         final Distance oldZ,
                                         final CoordinateTransformation oldC,
                                         final Speed oldSpeedX,
                                         final Speed oldSpeedY,
                                         final Speed oldSpeedZ,
                                         final Acceleration fx,
                                         final Acceleration fy,
                                         final Acceleration fz,
                                         final AngularSpeed angularRateX,
                                         final AngularSpeed angularRateY,
                                         final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public ECIFrame navigateAndReturnNew(final Time timeInterval,
                                         final Distance oldX,
                                         final Distance oldY,
                                         final Distance oldZ,
                                         final CoordinateTransformation oldC,
                                         final Speed oldSpeedX,
                                         final Speed oldSpeedY,
                                         final Speed oldSpeedZ,
                                         final Acceleration fx,
                                         final Acceleration fy,
                                         final Acceleration fz,
                                         final AngularSpeed angularRateX,
                                         final AngularSpeed angularRateY,
                                         final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public ECIFrame navigateAndReturnNew(final double timeInterval,
                                         final double oldX,
                                         final double oldY,
                                         final double oldZ,
                                         final CoordinateTransformation oldC,
                                         final double oldVx,
                                         final double oldVy,
                                         final double oldVz,
                                         final Acceleration fx,
                                         final Acceleration fy,
                                         final Acceleration fz,
                                         final AngularSpeed angularRateX,
                                         final AngularSpeed angularRateY,
                                         final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public ECIFrame navigateAndReturnNew(final Time timeInterval,
                                         final double oldX,
                                         final double oldY,
                                         final double oldZ,
                                         final CoordinateTransformation oldC,
                                         final double oldVx,
                                         final double oldVy,
                                         final double oldVz,
                                         final Acceleration fx,
                                         final Acceleration fy,
                                         final Acceleration fz,
                                         final AngularSpeed angularRateX,
                                         final AngularSpeed angularRateY,
                                         final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public ECIFrame navigateAndReturnNew(final double timeInterval,
                                         final Distance oldX,
                                         final Distance oldY,
                                         final Distance oldZ,
                                         final CoordinateTransformation oldC,
                                         final Speed oldSpeedX,
                                         final Speed oldSpeedY,
                                         final Speed oldSpeedZ,
                                         final BodyKinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public ECIFrame navigateAndReturnNew(final Time timeInterval,
                                         final Distance oldX,
                                         final Distance oldY,
                                         final Distance oldZ,
                                         final CoordinateTransformation oldC,
                                         final Speed oldSpeedX,
                                         final Speed oldSpeedY,
                                         final Speed oldSpeedZ,
                                         final BodyKinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        return navigateECIAndReturnNew(timeInterval, oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public ECIFrame navigateAndReturnNew(final double timeInterval,
                                         final ECIFrame oldFrame,
                                         final double fx,
                                         final double fy,
                                         final double fz,
                                         final double angularRateX,
                                         final double angularRateY,
                                         final double angularRateZ)
            throws InertialNavigatorException {
        return navigateECIAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public ECIFrame navigateAndReturnNew(final Time timeInterval,
                                         final ECIFrame oldFrame,
                                         final double fx,
                                         final double fy,
                                         final double fz,
                                         final double angularRateX,
                                         final double angularRateY,
                                         final double angularRateZ)
            throws InertialNavigatorException {
        return navigateECIAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public ECIFrame navigateAndReturnNew(final double timeInterval,
                                         final ECIFrame oldFrame,
                                         final BodyKinematics kinematics)
            throws InertialNavigatorException {
        return navigateECIAndReturnNew(timeInterval, oldFrame, kinematics);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public ECIFrame navigateAndReturnNew(final Time timeInterval,
                                         final ECIFrame oldFrame,
                                         final BodyKinematics kinematics)
            throws InertialNavigatorException {
        return navigateECIAndReturnNew(timeInterval, oldFrame, kinematics);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public ECIFrame navigateAndReturnNew(final double timeInterval,
                                         final ECIFrame oldFrame,
                                         final Acceleration fx,
                                         final Acceleration fy,
                                         final Acceleration fz,
                                         final double angularRateX,
                                         final double angularRateY,
                                         final double angularRateZ)
            throws InertialNavigatorException {
        return navigateECIAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public ECIFrame navigateAndReturnNew(final Time timeInterval,
                                         final ECIFrame oldFrame,
                                         final Acceleration fx,
                                         final Acceleration fy,
                                         final Acceleration fz,
                                         final double angularRateX,
                                         final double angularRateY,
                                         final double angularRateZ)
            throws InertialNavigatorException {
        return navigateECIAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public ECIFrame navigateAndReturnNew(final double timeInterval,
                                         final ECIFrame oldFrame,
                                         final double fx,
                                         final double fy,
                                         final double fz,
                                         final AngularSpeed angularRateX,
                                         final AngularSpeed angularRateY,
                                         final AngularSpeed angularRateZ)
            throws InertialNavigatorException {
        return navigateECIAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public ECIFrame navigateAndReturnNew(final Time timeInterval,
                                         final ECIFrame oldFrame,
                                         final double fx,
                                         final double fy,
                                         final double fz,
                                         final AngularSpeed angularRateX,
                                         final AngularSpeed angularRateY,
                                         final AngularSpeed angularRateZ)
            throws InertialNavigatorException {
        return navigateECIAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public ECIFrame navigateAndReturnNew(final double timeInterval,
                                         final ECIFrame oldFrame,
                                         final Acceleration fx,
                                         final Acceleration fy,
                                         final Acceleration fz,
                                         final AngularSpeed angularRateX,
                                         final AngularSpeed angularRateY,
                                         final AngularSpeed angularRateZ)
            throws InertialNavigatorException {
        return navigateECIAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public ECIFrame navigateAndReturnNew(final Time timeInterval,
                                         final ECIFrame oldFrame,
                                         final Acceleration fx,
                                         final Acceleration fy,
                                         final Acceleration fz,
                                         final AngularSpeed angularRateX,
                                         final AngularSpeed angularRateY,
                                         final AngularSpeed angularRateZ)
            throws InertialNavigatorException {
        return navigateECIAndReturnNew(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static void navigateECI(final double timeInterval,
                                   final double oldX,
                                   final double oldY,
                                   final double oldZ,
                                   final CoordinateTransformation oldC,
                                   final double oldVx,
                                   final double oldVy,
                                   final double oldVz,
                                   final double fx,
                                   final double fy,
                                   final double fz,
                                   final double angularRateX,
                                   final double angularRateY,
                                   final double angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {

        if (!isValidBodyToEciCoordinateTransformationMatrix(oldC)) {
            throw new InvalidSourceAndDestinationFrameTypeException();
        }

        try {
            // Attitude update
            // Calculate attitude increment, magnitude, and skew-symmetric matrix
            final Matrix alphaIbb = new Matrix(ROWS, 1);
            alphaIbb.setElementAtIndex(0, angularRateX * timeInterval);
            alphaIbb.setElementAtIndex(1, angularRateY * timeInterval);
            alphaIbb.setElementAtIndex(2, angularRateZ * timeInterval);

            final double magAlpha = Utils.normF(alphaIbb);
            final Matrix skewAlpha = Utils.skewMatrix(alphaIbb);

            // Obtain coordinate transformation matrix from the new attitude to the old
            // using Rodrigues' formula, (5.73)
            final Matrix cNewOld = Matrix.identity(ROWS, ROWS);
            if (magAlpha > ALPHA_THRESHOLD) {
                final double magAlpha2 = magAlpha * magAlpha;
                final double value1 = Math.sin(magAlpha) / magAlpha;
                final double value2 = (1.0 - Math.cos(magAlpha)) / magAlpha2;

                final Matrix tmp1 = skewAlpha.multiplyByScalarAndReturnNew(value1);
                final Matrix tmp2 = skewAlpha.multiplyByScalarAndReturnNew(value2);
                tmp2.multiply(skewAlpha);

                cNewOld.add(tmp1);
                cNewOld.add(tmp2);
            } else {
                cNewOld.add(skewAlpha);
            }

            // Update attitude
            final Matrix oldCbi = oldC.getMatrix();
            final Matrix cbi = oldCbi.multiplyAndReturnNew(cNewOld);

            // Specific force frame transformation
            // Calculate the average body-to-ECI-frame coordinate transformation
            // matrix over the update interval using (5.84)
            if (magAlpha > ALPHA_THRESHOLD) {
                final Matrix tmp1 = Matrix.identity(ROWS, ROWS);
                final double magAlpha2 = magAlpha * magAlpha;
                final double value1 = (1.0 - Math.cos(magAlpha)) / magAlpha2;
                final double value2 = (1.0 - Math.sin(magAlpha) / magAlpha) / magAlpha2;

                final Matrix tmp2 = skewAlpha.multiplyByScalarAndReturnNew(value1);
                final Matrix tmp3 = skewAlpha.multiplyByScalarAndReturnNew(value2);
                tmp3.multiply(skewAlpha);

                tmp1.add(tmp2);
                tmp1.add(tmp3);

                oldCbi.multiply(tmp1);
            }

            // oldCbi contains average body-to-ECI-frame (aveCbi)

            // Transform specific force to ECI-frame resolving axes using (5.81)
            final Matrix fIbb = new Matrix(ROWS, 1);
            fIbb.setElementAtIndex(0, fx);
            fIbb.setElementAtIndex(1, fy);
            fIbb.setElementAtIndex(2, fz);

            oldCbi.multiply(fIbb);
            // now oldCbi contains specific force to ECI-frame resolving axes(fIbi)

            // Update velocity
            // From (5.18) and (5.20),
            final Matrix oldVibi = new Matrix(ROWS, 1);
            oldVibi.setElementAtIndex(0, oldVx);
            oldVibi.setElementAtIndex(1, oldVy);
            oldVibi.setElementAtIndex(2, oldVz);

            final ECIGravitation gravitation = ECIGravitationEstimator
                    .estimateGravitationAndReturnNew(oldX, oldY, oldZ);
            final Matrix g = gravitation.asMatrix();

            //fIbi + g
            oldCbi.add(g);

            // timeInterval * (fIbi + g)
            oldCbi.multiplyByScalar(timeInterval);

            // oldVibi + timeInterval * (fIbi + g)
            final Matrix vIbi = oldVibi.addAndReturnNew(oldCbi);

            final double vx = vIbi.getElementAtIndex(0);
            final double vy = vIbi.getElementAtIndex(1);
            final double vz = vIbi.getElementAtIndex(2);

            // Update cartesian position
            // From (5.23),
            final double x = oldX + (vx + oldVx) * 0.5 * timeInterval;
            final double y = oldY + (vy + oldVy) * 0.5 * timeInterval;
            final double z = oldZ + (vz + oldVz) * 0.5 * timeInterval;

            final CoordinateTransformation newC = new CoordinateTransformation(cbi,
                    FrameType.BODY_FRAME, FrameType.EARTH_CENTERED_INERTIAL_FRAME);

            result.setCoordinates(x, y, z);
            result.setVelocityCoordinates(vx, vy, vz);
            result.setCoordinateTransformation(newC);

        } catch (final AlgebraException | InvalidRotationMatrixException e) {
            throw new InertialNavigatorException(e);
        }
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static void navigateECI(final Time timeInterval,
                                   final double oldX,
                                   final double oldY,
                                   final double oldZ,
                                   final CoordinateTransformation oldC,
                                   final double oldVx,
                                   final double oldVy,
                                   final double oldVz,
                                   final double fx,
                                   final double fy,
                                   final double fz,
                                   final double angularRateX,
                                   final double angularRateY,
                                   final double angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(convertTimeToDouble(timeInterval), oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static void navigateECI(final double timeInterval,
                                   final double oldX,
                                   final double oldY,
                                   final double oldZ,
                                   final CoordinateTransformation oldC,
                                   final double oldVx,
                                   final double oldVy,
                                   final double oldVz,
                                   final BodyKinematics kinematics,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(),
                kinematics.getAngularRateZ(), result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static void navigateECI(final Time timeInterval,
                                   final double oldX,
                                   final double oldY,
                                   final double oldZ,
                                   final CoordinateTransformation oldC,
                                   final double oldVx,
                                   final double oldVy,
                                   final double oldVz,
                                   final BodyKinematics kinematics,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(convertTimeToDouble(timeInterval), oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, kinematics, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous cartesian position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static void navigateECI(final double timeInterval,
                                   final Point3D oldPosition,
                                   final CoordinateTransformation oldC,
                                   final double oldVx,
                                   final double oldVy,
                                   final double oldVz,
                                   final double fx,
                                   final double fy,
                                   final double fz,
                                   final double angularRateX,
                                   final double angularRateY,
                                   final double angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldPosition.getInhomX(), oldPosition.getInhomY(), oldPosition.getInhomZ(),
                oldC, oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous cartesian position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static void navigateECI(final Time timeInterval,
                                   final Point3D oldPosition,
                                   final CoordinateTransformation oldC,
                                   final double oldVx,
                                   final double oldVy,
                                   final double oldVz,
                                   final double fx,
                                   final double fy,
                                   final double fz,
                                   final double angularRateX,
                                   final double angularRateY,
                                   final double angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldPosition.getInhomX(), oldPosition.getInhomY(), oldPosition.getInhomZ(),
                oldC, oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous cartesian position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static void navigateECI(final double timeInterval,
                                   final Point3D oldPosition,
                                   final CoordinateTransformation oldC,
                                   final double oldVx,
                                   final double oldVy,
                                   final double oldVz,
                                   final BodyKinematics kinematics,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz,
                kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(),
                kinematics.getAngularRateZ(), result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous cartesian position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static void navigateECI(final Time timeInterval,
                                   final Point3D oldPosition,
                                   final CoordinateTransformation oldC,
                                   final double oldVx,
                                   final double oldVy,
                                   final double oldVz,
                                   final BodyKinematics kinematics,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(convertTimeToDouble(timeInterval), oldPosition, oldC,
                oldVx, oldVy, oldVz, kinematics, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECI(final double timeInterval,
                                   final Distance oldX,
                                   final Distance oldY,
                                   final Distance oldZ,
                                   final CoordinateTransformation oldC,
                                   final double oldVx,
                                   final double oldVy,
                                   final double oldVz,
                                   final double fx,
                                   final double fy,
                                   final double fz,
                                   final double angularRateX,
                                   final double angularRateY,
                                   final double angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, convertDistanceToDouble(oldX),
                convertDistanceToDouble(oldY), convertDistanceToDouble(oldZ), oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY, angularRateZ,
                result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECI(final Time timeInterval,
                                   final Distance oldX,
                                   final Distance oldY,
                                   final Distance oldZ,
                                   final CoordinateTransformation oldC,
                                   final double oldVx,
                                   final double oldVy,
                                   final double oldVz,
                                   final double fx,
                                   final double fy,
                                   final double fz,
                                   final double angularRateX,
                                   final double angularRateY,
                                   final double angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(convertTimeToDouble(timeInterval), oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static void navigateECI(final double timeInterval,
                                   final Distance oldX,
                                   final Distance oldY,
                                   final Distance oldZ,
                                   final CoordinateTransformation oldC,
                                   final double oldVx,
                                   final double oldVy,
                                   final double oldVz,
                                   final BodyKinematics kinematics,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(),
                kinematics.getAngularRateZ(), result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static void navigateECI(final Time timeInterval,
                                   final Distance oldX,
                                   final Distance oldY,
                                   final Distance oldZ,
                                   final CoordinateTransformation oldC,
                                   final double oldVx,
                                   final double oldVy,
                                   final double oldVz,
                                   final BodyKinematics kinematics,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(),
                kinematics.getAngularRateZ(), result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECI(final double timeInterval,
                                   final double oldX,
                                   final double oldY,
                                   final double oldZ,
                                   final CoordinateTransformation oldC,
                                   final Speed oldSpeedX,
                                   final Speed oldSpeedY,
                                   final Speed oldSpeedZ,
                                   final double fx,
                                   final double fy,
                                   final double fz,
                                   final double angularRateX,
                                   final double angularRateY,
                                   final double angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC,
                convertSpeedToDouble(oldSpeedX), convertSpeedToDouble(oldSpeedY),
                convertSpeedToDouble(oldSpeedZ), fx, fy, fz, angularRateX,
                angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECI(final Time timeInterval,
                                   final double oldX,
                                   final double oldY,
                                   final double oldZ,
                                   final CoordinateTransformation oldC,
                                   final Speed oldSpeedX,
                                   final Speed oldSpeedY,
                                   final Speed oldSpeedZ,
                                   final double fx,
                                   final double fy,
                                   final double fz,
                                   final double angularRateX,
                                   final double angularRateY,
                                   final double angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(convertTimeToDouble(timeInterval), oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX,
                angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static void navigateECI(final double timeInterval,
                                   final double oldX,
                                   final double oldY,
                                   final double oldZ,
                                   final CoordinateTransformation oldC,
                                   final Speed oldSpeedX,
                                   final Speed oldSpeedY,
                                   final Speed oldSpeedZ,
                                   final BodyKinematics kinematics,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(),
                kinematics.getAngularRateZ(), result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static void navigateECI(final Time timeInterval,
                                   final double oldX,
                                   final double oldY,
                                   final double oldZ,
                                   final CoordinateTransformation oldC,
                                   final Speed oldSpeedX,
                                   final Speed oldSpeedY,
                                   final Speed oldSpeedZ,
                                   final BodyKinematics kinematics,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(convertTimeToDouble(timeInterval), oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECI(final double timeInterval,
                                   final double oldX,
                                   final double oldY,
                                   final double oldZ,
                                   final CoordinateTransformation oldC,
                                   final double oldVx,
                                   final double oldVy,
                                   final double oldVz,
                                   final Acceleration fx,
                                   final Acceleration fy,
                                   final Acceleration fz,
                                   final double angularRateX,
                                   final double angularRateY,
                                   final double angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                convertAccelerationToDouble(fx), convertAccelerationToDouble(fy),
                convertAccelerationToDouble(fz), angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECI(final Time timeInterval,
                                   final double oldX,
                                   final double oldY,
                                   final double oldZ,
                                   final CoordinateTransformation oldC,
                                   final double oldVx,
                                   final double oldVy,
                                   final double oldVz,
                                   final Acceleration fx,
                                   final Acceleration fy,
                                   final Acceleration fz,
                                   final double angularRateX,
                                   final double angularRateY,
                                   final double angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                convertAccelerationToDouble(fx), convertAccelerationToDouble(fy),
                convertAccelerationToDouble(fz), angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECI(final double timeInterval,
                                   final double oldX,
                                   final double oldY,
                                   final double oldZ,
                                   final CoordinateTransformation oldC,
                                   final double oldVx,
                                   final double oldVy,
                                   final double oldVz,
                                   final double fx,
                                   final double fy,
                                   final double fz,
                                   final AngularSpeed angularRateX,
                                   final AngularSpeed angularRateY,
                                   final AngularSpeed angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, convertAngularSpeedToDouble(angularRateX),
                convertAngularSpeedToDouble(angularRateY),
                convertAngularSpeedToDouble(angularRateZ), result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECI(final Time timeInterval,
                                   final double oldX,
                                   final double oldY,
                                   final double oldZ,
                                   final CoordinateTransformation oldC,
                                   final double oldVx,
                                   final double oldVy,
                                   final double oldVz,
                                   final double fx,
                                   final double fy,
                                   final double fz,
                                   final AngularSpeed angularRateX,
                                   final AngularSpeed angularRateY,
                                   final AngularSpeed angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(convertTimeToDouble(timeInterval), oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECI(final double timeInterval,
                                   final Distance oldX,
                                   final Distance oldY,
                                   final Distance oldZ,
                                   final CoordinateTransformation oldC,
                                   final Speed oldSpeedX,
                                   final Speed oldSpeedY,
                                   final Speed oldSpeedZ,
                                   final double fx,
                                   final double fy,
                                   final double fz,
                                   final double angularRateX,
                                   final double angularRateY,
                                   final double angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, convertDistanceToDouble(oldX),
                convertDistanceToDouble(oldY), convertDistanceToDouble(oldZ), oldC,
                convertSpeedToDouble(oldSpeedX), convertSpeedToDouble(oldSpeedY),
                convertSpeedToDouble(oldSpeedZ), fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECI(final Time timeInterval,
                                   final Distance oldX,
                                   final Distance oldY,
                                   final Distance oldZ,
                                   final CoordinateTransformation oldC,
                                   final Speed oldSpeedX,
                                   final Speed oldSpeedY,
                                   final Speed oldSpeedZ,
                                   final double fx,
                                   final double fy,
                                   final double fz,
                                   final double angularRateX,
                                   final double angularRateY,
                                   final double angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(convertTimeToDouble(timeInterval), oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECI(final double timeInterval,
                                   final Distance oldX,
                                   final Distance oldY,
                                   final Distance oldZ,
                                   final CoordinateTransformation oldC,
                                   final Speed oldSpeedX,
                                   final Speed oldSpeedY,
                                   final Speed oldSpeedZ,
                                   final Acceleration fx,
                                   final Acceleration fy,
                                   final Acceleration fz,
                                   final AngularSpeed angularRateX,
                                   final AngularSpeed angularRateY,
                                   final AngularSpeed angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, convertDistanceToDouble(oldX),
                convertDistanceToDouble(oldY), convertDistanceToDouble(oldZ), oldC,
                convertSpeedToDouble(oldSpeedX), convertSpeedToDouble(oldSpeedY),
                convertSpeedToDouble(oldSpeedZ), convertAccelerationToDouble(fx),
                convertAccelerationToDouble(fy), convertAccelerationToDouble(fz),
                convertAngularSpeedToDouble(angularRateX),
                convertAngularSpeedToDouble(angularRateY),
                convertAngularSpeedToDouble(angularRateZ),
                result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECI(final Time timeInterval,
                                   final Distance oldX,
                                   final Distance oldY,
                                   final Distance oldZ,
                                   final CoordinateTransformation oldC,
                                   final Speed oldSpeedX,
                                   final Speed oldSpeedY,
                                   final Speed oldSpeedZ,
                                   final Acceleration fx,
                                   final Acceleration fy,
                                   final Acceleration fz,
                                   final AngularSpeed angularRateX,
                                   final AngularSpeed angularRateY,
                                   final AngularSpeed angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(convertTimeToDouble(timeInterval), oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, fx, fy, fz, angularRateX,
                angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECI(final double timeInterval,
                                   final double oldX,
                                   final double oldY,
                                   final double oldZ,
                                   final CoordinateTransformation oldC,
                                   final double oldVx,
                                   final double oldVy,
                                   final double oldVz,
                                   final Acceleration fx,
                                   final Acceleration fy,
                                   final Acceleration fz,
                                   final AngularSpeed angularRateX,
                                   final AngularSpeed angularRateY,
                                   final AngularSpeed angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                convertAccelerationToDouble(fx), convertAccelerationToDouble(fy),
                convertAccelerationToDouble(fz),
                convertAngularSpeedToDouble(angularRateX),
                convertAngularSpeedToDouble(angularRateY),
                convertAngularSpeedToDouble(angularRateZ), result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static void navigateECI(final Time timeInterval,
                                   final double oldX,
                                   final double oldY,
                                   final double oldZ,
                                   final CoordinateTransformation oldC,
                                   final double oldVx,
                                   final double oldVy,
                                   final double oldVz,
                                   final Acceleration fx,
                                   final Acceleration fy,
                                   final Acceleration fz,
                                   final AngularSpeed angularRateX,
                                   final AngularSpeed angularRateY,
                                   final AngularSpeed angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(convertTimeToDouble(timeInterval), oldX, oldY, oldZ, oldC,
                oldVx, oldVy, oldVz, fx, fy, fz, angularRateX, angularRateY,
                angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static void navigateECI(final double timeInterval,
                                   final Distance oldX,
                                   final Distance oldY,
                                   final Distance oldZ,
                                   final CoordinateTransformation oldC,
                                   final Speed oldSpeedX,
                                   final Speed oldSpeedY,
                                   final Speed oldSpeedZ,
                                   final BodyKinematics kinematics,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY,
                oldSpeedZ, kinematics.getFx(), kinematics.getFy(), kinematics.getFz(),
                kinematics.getAngularRateX(), kinematics.getAngularRateY(),
                kinematics.getAngularRateZ(), result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static void navigateECI(final Time timeInterval,
                                   final Distance oldX,
                                   final Distance oldY,
                                   final Distance oldZ,
                                   final CoordinateTransformation oldC,
                                   final Speed oldSpeedX,
                                   final Speed oldSpeedY,
                                   final Speed oldSpeedZ,
                                   final BodyKinematics kinematics,
                                   final ECIFrame result)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        navigateECI(convertTimeToDouble(timeInterval), oldX, oldY, oldZ, oldC,
                oldSpeedX, oldSpeedY, oldSpeedZ, kinematics, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static void navigateECI(final double timeInterval,
                                   final ECIFrame oldFrame,
                                   final double fx,
                                   final double fy,
                                   final double fz,
                                   final double angularRateX,
                                   final double angularRateY,
                                   final double angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException {
        try {
            navigateECI(timeInterval, oldFrame.getX(), oldFrame.getY(), oldFrame.getZ(),
                    oldFrame.getCoordinateTransformation(),
                    oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(), fx, fy, fz,
                    angularRateX, angularRateY, angularRateZ, result);
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
            // never happens
        }
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static void navigateECI(final Time timeInterval,
                                   final ECIFrame oldFrame,
                                   final double fx,
                                   final double fy,
                                   final double fz,
                                   final double angularRateX,
                                   final double angularRateY,
                                   final double angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException {
        navigateECI(convertTimeToDouble(timeInterval), oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static void navigateECI(final double timeInterval,
                                   final ECIFrame oldFrame,
                                   final BodyKinematics kinematics,
                                   final ECIFrame result)
            throws InertialNavigatorException {
        try {
            navigateECI(timeInterval, oldFrame.getX(), oldFrame.getY(), oldFrame.getZ(),
                    oldFrame.getCoordinateTransformation(),
                    oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(), kinematics,
                    result);
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
            // never happens
        }
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static void navigateECI(final Time timeInterval,
                                   final ECIFrame oldFrame,
                                   final BodyKinematics kinematics,
                                   final ECIFrame result)
            throws InertialNavigatorException {
        navigateECI(convertTimeToDouble(timeInterval), oldFrame, kinematics, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static void navigateECI(final double timeInterval,
                                   final ECIFrame oldFrame,
                                   final Acceleration fx,
                                   final Acceleration fy,
                                   final Acceleration fz,
                                   final double angularRateX,
                                   final double angularRateY,
                                   final double angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException {
        try {
            navigateECI(timeInterval, oldFrame.getX(), oldFrame.getY(), oldFrame.getZ(),
                    oldFrame.getCoordinateTransformation(),
                    oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(), fx, fy, fz,
                    angularRateX, angularRateY, angularRateZ, result);
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
            // never happens
        }
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static void navigateECI(final Time timeInterval,
                                   final ECIFrame oldFrame,
                                   final Acceleration fx,
                                   final Acceleration fy,
                                   final Acceleration fz,
                                   final double angularRateX,
                                   final double angularRateY,
                                   final double angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException {
        navigateECI(convertTimeToDouble(timeInterval), oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static void navigateECI(final double timeInterval,
                                   final ECIFrame oldFrame,
                                   final double fx,
                                   final double fy,
                                   final double fz,
                                   final AngularSpeed angularRateX,
                                   final AngularSpeed angularRateY,
                                   final AngularSpeed angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException {
        try {
            navigateECI(timeInterval, oldFrame.getX(), oldFrame.getY(), oldFrame.getZ(),
                    oldFrame.getCoordinateTransformation(),
                    oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(), fx, fy, fz,
                    angularRateX, angularRateY, angularRateZ, result);
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
            // never happens
        }
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static void navigateECI(final Time timeInterval,
                                   final ECIFrame oldFrame,
                                   final double fx,
                                   final double fy,
                                   final double fz,
                                   final AngularSpeed angularRateX,
                                   final AngularSpeed angularRateY,
                                   final AngularSpeed angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException {
        navigateECI(convertTimeToDouble(timeInterval), oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static void navigateECI(final double timeInterval,
                                   final ECIFrame oldFrame,
                                   final Acceleration fx,
                                   final Acceleration fy,
                                   final Acceleration fz,
                                   final AngularSpeed angularRateX,
                                   final AngularSpeed angularRateY,
                                   final AngularSpeed angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException {
        try {
            navigateECI(timeInterval, oldFrame.getX(), oldFrame.getY(), oldFrame.getZ(),
                    oldFrame.getCoordinateTransformation(),
                    oldFrame.getVx(), oldFrame.getVy(), oldFrame.getVz(), fx, fy, fz,
                    angularRateX, angularRateY, angularRateZ, result);
        } catch (final InvalidSourceAndDestinationFrameTypeException ignore) {
            // never happens
        }
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param result       instance where new estimated ECI frame containing new body position,
     *                     velocity and coordinate transformation matrix will be stored.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static void navigateECI(final Time timeInterval,
                                   final ECIFrame oldFrame,
                                   final Acceleration fx,
                                   final Acceleration fy,
                                   final Acceleration fz,
                                   final AngularSpeed angularRateX,
                                   final AngularSpeed angularRateY,
                                   final AngularSpeed angularRateZ,
                                   final ECIFrame result)
            throws InertialNavigatorException {
        navigateECI(convertTimeToDouble(timeInterval), oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final double timeInterval,
                                                   final double oldX,
                                                   final double oldY,
                                                   final double oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final double oldVx,
                                                   final double oldVy,
                                                   final double oldVz,
                                                   final double fx,
                                                   final double fy,
                                                   final double fz,
                                                   final double angularRateX,
                                                   final double angularRateY,
                                                   final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final Time timeInterval,
                                                   final double oldX,
                                                   final double oldY,
                                                   final double oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final double oldVx,
                                                   final double oldVy,
                                                   final double oldVz,
                                                   final double fx,
                                                   final double fy,
                                                   final double fz,
                                                   final double angularRateX,
                                                   final double angularRateY,
                                                   final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final double timeInterval,
                                                   final double oldX,
                                                   final double oldY,
                                                   final double oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final double oldVx,
                                                   final double oldVy,
                                                   final double oldVz,
                                                   final BodyKinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, kinematics,
                result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final Time timeInterval,
                                                   final double oldX,
                                                   final double oldY,
                                                   final double oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final double oldVx,
                                                   final double oldVy,
                                                   final double oldVz,
                                                   final BodyKinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, kinematics,
                result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous cartesian position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final double timeInterval,
                                                   final Point3D oldPosition,
                                                   final CoordinateTransformation oldC,
                                                   final double oldVx,
                                                   final double oldVy,
                                                   final double oldVz,
                                                   final double fx,
                                                   final double fy,
                                                   final double fz,
                                                   final double angularRateX,
                                                   final double angularRateY,
                                                   final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous cartesian position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final Time timeInterval,
                                                   final Point3D oldPosition,
                                                   final CoordinateTransformation oldC,
                                                   final double oldVx,
                                                   final double oldVy,
                                                   final double oldVz,
                                                   final double fx,
                                                   final double fy,
                                                   final double fz,
                                                   final double angularRateX,
                                                   final double angularRateY,
                                                   final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldPosition  previous cartesian position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final double timeInterval,
                                                   final Point3D oldPosition,
                                                   final CoordinateTransformation oldC,
                                                   final double oldVx,
                                                   final double oldVy,
                                                   final double oldVz,
                                                   final BodyKinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz, kinematics,
                result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldPosition  previous cartesian position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final Time timeInterval,
                                                   final Point3D oldPosition,
                                                   final CoordinateTransformation oldC,
                                                   final double oldVx,
                                                   final double oldVy,
                                                   final double oldVz,
                                                   final BodyKinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldPosition, oldC, oldVx, oldVy, oldVz, kinematics,
                result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final double timeInterval,
                                                   final Distance oldX,
                                                   final Distance oldY,
                                                   final Distance oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final double oldVx,
                                                   final double oldVy,
                                                   final double oldVz,
                                                   final double fx,
                                                   final double fy,
                                                   final double fz,
                                                   final double angularRateX,
                                                   final double angularRateY,
                                                   final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final Time timeInterval,
                                                   final Distance oldX,
                                                   final Distance oldY,
                                                   final Distance oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final double oldVx,
                                                   final double oldVy,
                                                   final double oldVz,
                                                   final double fx,
                                                   final double fy,
                                                   final double fz,
                                                   final double angularRateX,
                                                   final double angularRateY,
                                                   final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final double timeInterval,
                                                   final Distance oldX,
                                                   final Distance oldY,
                                                   final Distance oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final double oldVx,
                                                   final double oldVy,
                                                   final double oldVz,
                                                   final BodyKinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, kinematics,
                result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final Time timeInterval,
                                                   final Distance oldX,
                                                   final Distance oldY,
                                                   final Distance oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final double oldVx,
                                                   final double oldVy,
                                                   final double oldVz,
                                                   final BodyKinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz, kinematics,
                result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final double timeInterval,
                                                   final double oldX,
                                                   final double oldY,
                                                   final double oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final Speed oldSpeedX,
                                                   final Speed oldSpeedY,
                                                   final Speed oldSpeedZ,
                                                   final double fx,
                                                   final double fy,
                                                   final double fz,
                                                   final double angularRateX,
                                                   final double angularRateY,
                                                   final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final Time timeInterval,
                                                   final double oldX,
                                                   final double oldY,
                                                   final double oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final Speed oldSpeedX,
                                                   final Speed oldSpeedY,
                                                   final Speed oldSpeedZ,
                                                   final double fx,
                                                   final double fy,
                                                   final double fz,
                                                   final double angularRateX,
                                                   final double angularRateY,
                                                   final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final double timeInterval,
                                                   final double oldX,
                                                   final double oldY,
                                                   final double oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final Speed oldSpeedX,
                                                   final Speed oldSpeedY,
                                                   final Speed oldSpeedZ,
                                                   final BodyKinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final Time timeInterval,
                                                   final double oldX,
                                                   final double oldY,
                                                   final double oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final Speed oldSpeedX,
                                                   final Speed oldSpeedY,
                                                   final Speed oldSpeedZ,
                                                   final BodyKinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final double timeInterval,
                                                   final double oldX,
                                                   final double oldY,
                                                   final double oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final double oldVx,
                                                   final double oldVy,
                                                   final double oldVz,
                                                   final Acceleration fx,
                                                   final Acceleration fy,
                                                   final Acceleration fz,
                                                   final double angularRateX,
                                                   final double angularRateY,
                                                   final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final Time timeInterval,
                                                   final double oldX,
                                                   final double oldY,
                                                   final double oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final double oldVx,
                                                   final double oldVy,
                                                   final double oldVz,
                                                   final Acceleration fx,
                                                   final Acceleration fy,
                                                   final Acceleration fz,
                                                   final double angularRateX,
                                                   final double angularRateY,
                                                   final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final double timeInterval,
                                                   final double oldX,
                                                   final double oldY,
                                                   final double oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final double oldVx,
                                                   final double oldVy,
                                                   final double oldVz,
                                                   final double fx,
                                                   final double fy,
                                                   final double fz,
                                                   final AngularSpeed angularRateX,
                                                   final AngularSpeed angularRateY,
                                                   final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final Time timeInterval,
                                                   final double oldX,
                                                   final double oldY,
                                                   final double oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final double oldVx,
                                                   final double oldVy,
                                                   final double oldVz,
                                                   final double fx,
                                                   final double fy,
                                                   final double fz,
                                                   final AngularSpeed angularRateX,
                                                   final AngularSpeed angularRateY,
                                                   final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final double timeInterval,
                                                   final Distance oldX,
                                                   final Distance oldY,
                                                   final Distance oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final Speed oldSpeedX,
                                                   final Speed oldSpeedY,
                                                   final Speed oldSpeedZ,
                                                   final double fx,
                                                   final double fy,
                                                   final double fz,
                                                   final double angularRateX,
                                                   final double angularRateY,
                                                   final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final Time timeInterval,
                                                   final Distance oldX,
                                                   final Distance oldY,
                                                   final Distance oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final Speed oldSpeedX,
                                                   final Speed oldSpeedY,
                                                   final Speed oldSpeedZ,
                                                   final double fx,
                                                   final double fy,
                                                   final double fz,
                                                   final double angularRateX,
                                                   final double angularRateY,
                                                   final double angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final double timeInterval,
                                                   final Distance oldX,
                                                   final Distance oldY,
                                                   final Distance oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final Speed oldSpeedX,
                                                   final Speed oldSpeedY,
                                                   final Speed oldSpeedZ,
                                                   final Acceleration fx,
                                                   final Acceleration fy,
                                                   final Acceleration fz,
                                                   final AngularSpeed angularRateX,
                                                   final AngularSpeed angularRateY,
                                                   final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final Time timeInterval,
                                                   final Distance oldX,
                                                   final Distance oldY,
                                                   final Distance oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final Speed oldSpeedX,
                                                   final Speed oldSpeedY,
                                                   final Speed oldSpeedZ,
                                                   final Acceleration fx,
                                                   final Acceleration fy,
                                                   final Acceleration fz,
                                                   final AngularSpeed angularRateX,
                                                   final AngularSpeed angularRateY,
                                                   final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final double timeInterval,
                                                   final double oldX,
                                                   final double oldY,
                                                   final double oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final double oldVx,
                                                   final double oldVy,
                                                   final double oldVz,
                                                   final Acceleration fx,
                                                   final Acceleration fy,
                                                   final Acceleration fz,
                                                   final AngularSpeed angularRateX,
                                                   final AngularSpeed angularRateY,
                                                   final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes and expressed in meters (m).
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldVx        previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVy        previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param oldVz        previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes and expressed in meters per second (m/s).
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final Time timeInterval,
                                                   final double oldX,
                                                   final double oldY,
                                                   final double oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final double oldVx,
                                                   final double oldVy,
                                                   final double oldVz,
                                                   final Acceleration fx,
                                                   final Acceleration fy,
                                                   final Acceleration fz,
                                                   final AngularSpeed angularRateX,
                                                   final AngularSpeed angularRateY,
                                                   final AngularSpeed angularRateZ)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldVx, oldVy, oldVz,
                fx, fy, fz, angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final double timeInterval,
                                                   final Distance oldX,
                                                   final Distance oldY,
                                                   final Distance oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final Speed oldSpeedX,
                                                   final Speed oldSpeedY,
                                                   final Speed oldSpeedZ,
                                                   final BodyKinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldX         previous cartesian x-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldY         previous cartesian y-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldZ         previous cartesian z-coordinate position of body frame with respect ECI
     *                     frame, resolved along ECI-frame axes.
     * @param oldC         previous body-to-ECI-frame coordinate transformation.
     * @param oldSpeedX    previous velocity x-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedY    previous velocity y-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param oldSpeedZ    previous velocity z-coordinate of body frame with respect ECI frame,
     *                     resolved along ECI-frame axes.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException                    if navigation fails due to numerical instabilities.
     * @throws InvalidSourceAndDestinationFrameTypeException if source or destination frame types of previous
     *                                                       body-to-ECI-frame coordinate transformation matrix are
     *                                                       invalid.
     */
    public static ECIFrame navigateECIAndReturnNew(final Time timeInterval,
                                                   final Distance oldX,
                                                   final Distance oldY,
                                                   final Distance oldZ,
                                                   final CoordinateTransformation oldC,
                                                   final Speed oldSpeedX,
                                                   final Speed oldSpeedY,
                                                   final Speed oldSpeedZ,
                                                   final BodyKinematics kinematics)
            throws InertialNavigatorException, InvalidSourceAndDestinationFrameTypeException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldX, oldY, oldZ, oldC, oldSpeedX, oldSpeedY, oldSpeedZ,
                kinematics, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static ECIFrame navigateECIAndReturnNew(final double timeInterval,
                                                   final ECIFrame oldFrame,
                                                   final double fx,
                                                   final double fy,
                                                   final double fz,
                                                   final double angularRateX,
                                                   final double angularRateY,
                                                   final double angularRateZ)
            throws InertialNavigatorException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static ECIFrame navigateECIAndReturnNew(final Time timeInterval,
                                                   final ECIFrame oldFrame,
                                                   final double fx,
                                                   final double fy,
                                                   final double fz,
                                                   final double angularRateX,
                                                   final double angularRateY,
                                                   final double angularRateZ)
            throws InertialNavigatorException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static ECIFrame navigateECIAndReturnNew(final double timeInterval,
                                                   final ECIFrame oldFrame,
                                                   final BodyKinematics kinematics)
            throws InertialNavigatorException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldFrame, kinematics, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param kinematics   body kinematics containing specific forces and angular rates applied to
     *                     the body.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static ECIFrame navigateECIAndReturnNew(final Time timeInterval,
                                                   final ECIFrame oldFrame,
                                                   final BodyKinematics kinematics)
            throws InertialNavigatorException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldFrame, kinematics, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static ECIFrame navigateECIAndReturnNew(final double timeInterval,
                                                   final ECIFrame oldFrame,
                                                   final Acceleration fx,
                                                   final Acceleration fy,
                                                   final Acceleration fz,
                                                   final double angularRateX,
                                                   final double angularRateY,
                                                   final double angularRateZ)
            throws InertialNavigatorException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in radians per second (rad/s).
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static ECIFrame navigateECIAndReturnNew(final Time timeInterval,
                                                   final ECIFrame oldFrame,
                                                   final Acceleration fx,
                                                   final Acceleration fy,
                                                   final Acceleration fz,
                                                   final double angularRateX,
                                                   final double angularRateY,
                                                   final double angularRateZ)
            throws InertialNavigatorException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static ECIFrame navigateECIAndReturnNew(final double timeInterval,
                                                   final ECIFrame oldFrame,
                                                   final double fx,
                                                   final double fy,
                                                   final double fz,
                                                   final AngularSpeed angularRateX,
                                                   final AngularSpeed angularRateY,
                                                   final AngularSpeed angularRateZ)
            throws InertialNavigatorException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval and
     *                     expressed in meters per squared second (m/s^2).
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static ECIFrame navigateECIAndReturnNew(final Time timeInterval,
                                                   final ECIFrame oldFrame,
                                                   final double fx,
                                                   final double fy,
                                                   final double fz,
                                                   final AngularSpeed angularRateX,
                                                   final AngularSpeed angularRateY,
                                                   final AngularSpeed angularRateZ)
            throws InertialNavigatorException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs expressed in seconds (s).
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static ECIFrame navigateECIAndReturnNew(final double timeInterval,
                                                   final ECIFrame oldFrame,
                                                   final Acceleration fx,
                                                   final Acceleration fy,
                                                   final Acceleration fz,
                                                   final AngularSpeed angularRateX,
                                                   final AngularSpeed angularRateY,
                                                   final AngularSpeed angularRateZ)
            throws InertialNavigatorException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Runs precision ECI-frame inertial navigation equations.
     *
     * @param timeInterval time interval between epochs.
     * @param oldFrame     previous ECI frame containing body position, velocity and
     *                     coordinate transformation matrix.
     * @param fx           specific force x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fy           specific force y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param fz           specific force z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateX angular rate x-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateY angular rate y-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @param angularRateZ angular rate z-coordinate of body frame with respect ECI frame,
     *                     resolved along body-frame axes, averaged over time interval.
     * @return estimated ECI frame containing new body position, velocity and coordinate
     * transformation matrix.
     * @throws InertialNavigatorException if navigation fails due to numerical instabilities.
     */
    public static ECIFrame navigateECIAndReturnNew(final Time timeInterval,
                                                   final ECIFrame oldFrame,
                                                   final Acceleration fx,
                                                   final Acceleration fy,
                                                   final Acceleration fz,
                                                   final AngularSpeed angularRateX,
                                                   final AngularSpeed angularRateY,
                                                   final AngularSpeed angularRateZ)
            throws InertialNavigatorException {
        final ECIFrame result = new ECIFrame();
        navigateECI(timeInterval, oldFrame, fx, fy, fz,
                angularRateX, angularRateY, angularRateZ, result);
        return result;
    }

    /**
     * Checks whether provided coordinate transformation matrix is valid or not.
     * Only body to ECI transformation matrices are considered to be valid.
     *
     * @param c coordinate transformation matrix to be checked.
     * @return true if provided value is valid, false otherwise.
     */
    public static boolean isValidBodyToEciCoordinateTransformationMatrix(final CoordinateTransformation c) {
        return ECIFrame.isValidCoordinateTransformation(c);
    }

    /**
     * Converts provided time instance into its corresponding value expressed in
     * seconds.
     *
     * @param time time instance to be converted.
     * @return converted value expressed in seconds.
     */
    private static double convertTimeToDouble(final com.irurueta.units.Time time) {
        return TimeConverter.convert(time.getValue().doubleValue(), time.getUnit(),
                TimeUnit.SECOND);
    }

    /**
     * Converts provided distance instance into its corresponding value expressed in
     * meters.
     *
     * @param distance distance instance to be converted.
     * @return converted value expressed in meters.
     */
    private static double convertDistanceToDouble(final Distance distance) {
        return DistanceConverter.convert(distance.getValue().doubleValue(),
                distance.getUnit(), DistanceUnit.METER);
    }

    /**
     * Converts provided speed instance into its corresponding value expressed in
     * meters per second.
     *
     * @param speed speed instance to be converted.
     * @return converted value expressed in meters per second.
     */
    private static double convertSpeedToDouble(final Speed speed) {
        return SpeedConverter.convert(speed.getValue().doubleValue(),
                speed.getUnit(), SpeedUnit.METERS_PER_SECOND);
    }

    /**
     * Converts provided acceleration instance into its corresponding value expressed
     * in meters per squared second.
     *
     * @param acceleration acceleration instance to be converted.
     * @return converted value expressed in meters per squared second.
     */
    private static double convertAccelerationToDouble(final Acceleration acceleration) {
        return AccelerationConverter.convert(acceleration.getValue().doubleValue(),
                acceleration.getUnit(), AccelerationUnit.METERS_PER_SQUARED_SECOND);
    }

    /**
     * Converts provided angular speed into its corresponding value expressed in
     * radians per second.
     *
     * @param angularSpeed angular speed instance to be converted.
     * @return converted value expressed in radians per second.
     */
    private static double convertAngularSpeedToDouble(final AngularSpeed angularSpeed) {
        return AngularSpeedConverter.convert(angularSpeed.getValue().doubleValue(),
                angularSpeed.getUnit(), AngularSpeedUnit.RADIANS_PER_SECOND);
    }
}
